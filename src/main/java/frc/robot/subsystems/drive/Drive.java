// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Arrays;
import java.util.Objects;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.stream.IntStream;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.RobotState;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.drive.DriveConstants.ModuleConstants;
import frc.robot.subsystems.leds.Leds;
import frc.util.AllianceFlipUtil;
import frc.util.LazyOptional;
import frc.util.LoggedTunableNumber;
import frc.util.MathExtraUtil;
import frc.util.Perspective;
import frc.util.VirtualSubsystem;
import frc.util.controllers.Joystick;
import frc.util.robotStructure.Root;

public class Drive extends VirtualSubsystem {
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private Rotation2d gyroAngle = new Rotation2d();

    public final Root structureRoot = new Root();

    public final Module[] modules = new Module[DriveConstants.moduleConstants.length];

    private SwerveModuleState[] measuredStates = new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
    };
    private ChassisSpeeds robotMeasuredSpeeds = new ChassisSpeeds();
    private ChassisSpeeds fieldMeasuredSpeeds = new ChassisSpeeds();

    private static final LoggedTunableNumber rotationCorrection = new LoggedTunableNumber("Drive/Rotation Correction", 0.125);

    private ChassisSpeeds setpointSpeeds = new ChassisSpeeds();
    private Translation2d centerOfRotation = new Translation2d();
    private SwerveModuleState[] setpointStates = new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
    };

    private Twist2d fieldVelocity = new Twist2d();

    public Drive(GyroIO gyroIO, ModuleIO... moduleIOs) {
        System.out.println("[Init Drive] Instantiating Drive");
        this.gyroIO = gyroIO;
        System.out.println("[Init Drive] Gyro IO: " + this.gyroIO.getClass().getSimpleName());
        for(int i = 0; i < DriveConstants.moduleConstants.length; i++) {
            ModuleConstants config = DriveConstants.moduleConstants[i];
            System.out.println("[Init Drive] Instantiating Module " + config.name + " with Module IO: " + moduleIOs[i].getClass().getSimpleName());
            var module = new Module(moduleIOs[i], config);
            module.setBrakeMode(false);
            module.periodic();
            modules[i] = module;
        }

        Pose2d initialPose = new Pose2d();
        RobotState.getInstance().initializePoseEstimator(DriveConstants.kinematics, getGyroRotation(), getModulePositions(), initialPose);
        gyroAngle = getPose().getRotation();

        this.translationSubsystem = new Translational(this);
        this.rotationalSubsystem = new Rotational(this);
        AutoBuilder.configure(
            this::getPose,
            this::setPose,
            this::getRobotMeasuredSpeeds,
            this::driveVelocity,
            autoConfig(),
            robotConfig(),
            AllianceFlipUtil::shouldFlip,
            translationSubsystem,
            rotationalSubsystem
        );

        var routine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> {
                    Logger.recordOutput("SysID/Drive/State", state.toString());
                }
            ),
            new SysIdRoutine.Mechanism(
                (volts) -> {
                    Arrays.stream(modules).forEach((module) -> module.runVoltage(volts, Rotation2d.kZero));
                },
                (log) -> {
                    Arrays.stream(modules).forEach((module) -> {
                        Logger.recordOutput("SysID/Drive/" + module.config.name + "/Position", module.getWheelAngularPosition());
                        Logger.recordOutput("SysID/Drive/" + module.config.name + "/Velocity", module.getWheelAngularVelocity());
                        Logger.recordOutput("SysID/Drive/" + module.config.name + "/Voltage", module.getAppliedVoltage());
                    });
                },
                this.translationSubsystem
            )
        );

        SmartDashboard.putData("SysID/Drive/Quasi Forward", routine.quasistatic(Direction.kForward).alongWith(Commands.idle(this.rotationalSubsystem)).withName("SysID Quasistatic Forward").asProxy());
        SmartDashboard.putData("SysID/Drive/Quasi Reverse", routine.quasistatic(Direction.kReverse).alongWith(Commands.idle(this.rotationalSubsystem)).withName("SysID Quasistatic Reverse").asProxy());
        SmartDashboard.putData("SysID/Drive/Dynamic Forward", routine.dynamic(Direction.kForward).alongWith(Commands.idle(this.rotationalSubsystem)).withName("SysID Dynamic Forward").asProxy());
        SmartDashboard.putData("SysID/Drive/Dynamic Reverse", routine.dynamic(Direction.kReverse).alongWith(Commands.idle(this.rotationalSubsystem)).withName("SysID Dynamic Reverse").asProxy());
    }

    private static final SwerveModuleState[] emptyStates = new SwerveModuleState[0];
    private static final ChassisSpeeds emptySpeeds = new ChassisSpeeds();
    public void periodic() {
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Inputs/Drive/Gyro", gyroInputs);
        Arrays.stream(modules).forEach(Module::periodic);

        measuredStates = Arrays.stream(modules).map(Module::getModuleState).toArray(SwerveModuleState[]::new);
        Logger.recordOutput("Drive/Swerve States/Measured", measuredStates);

        // Update odometry
        // Update field velocity
        robotMeasuredSpeeds = DriveConstants.kinematics.toChassisSpeeds(measuredStates);
        if (gyroInputs.connected) {
            gyroAngle = getGyroRotation();
            robotMeasuredSpeeds.omegaRadiansPerSecond = gyroInputs.yawVelocity.in(RadiansPerSecond);
        } else {
            // either the gyro is disconnected or we are in a simulation
            // accumulate a gyro estimate using module kinematics
            var wheelDeltas = getModulePositionDeltas(); // get change in module positions
            Twist2d twist = DriveConstants.kinematics.toTwist2d(wheelDeltas); // dtheta will be the estimated change in chassis angle
            gyroAngle = gyroAngle.plus(Rotation2d.fromRadians(twist.dtheta));
        }
        Logger.recordOutput("Drive/Chassis Speeds/Measured", robotMeasuredSpeeds);
        RobotState.getInstance().addDriveMeasurement(gyroAngle, getModulePositions());
        structureRoot.setPose(getPose());
        fieldMeasuredSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotMeasuredSpeeds, gyroAngle);

        // Skid Detection
        // SwerveModuleState[] rotationalStates = new SwerveModuleState[DriveConstants.modules.length];
        // SwerveModuleState[] translationalStates = new SwerveModuleState[DriveConstants.modules.length];

        // for (int i = 0; i < DriveConstants.modules.length; i++) {
        //     var rotationalState = new SwerveModuleState(-gyroInputs.yawVelocity.in(RadiansPerSecond) * DriveConstants.modules[i].moduleTranslation.getNorm(), MathExtraUtil.rotationFromVector(DriveConstants.modules[i].positiveRotVec));
        //     var rotational = new Translation2d(rotationalState.speedMetersPerSecond, rotationalState.angle);
        //     rotationalStates[i] = rotationalState;
        //     var measured = new Translation2d(measuredStates[i].speedMetersPerSecond, measuredStates[i].angle);
        //     var translational = measured.minus(rotational);
        //     translationalStates[i] = new SwerveModuleState(translational.getNorm(), translational.getAngle());
        // }

        // Logger.recordOutput("Drive/SwerveStates/Rotational States", rotationalStates);
        // Logger.recordOutput("Drive/SwerveStates/Translational States", translationalStates);

        // var minTranslational = Arrays.stream(translationalStates).mapToDouble((state) -> state.speedMetersPerSecond).map(Math::abs).min().orElse(0);
        // var maxTranslational = Arrays.stream(translationalStates).mapToDouble((state) -> state.speedMetersPerSecond).map(Math::abs).max().orElse(0);
        // var averageTranslational = Arrays.stream(translationalStates).mapToDouble((state) -> state.speedMetersPerSecond).map(Math::abs).average().orElse(0);
        // var maxDistanceFromAverage = Arrays.stream(translationalStates).mapToDouble((state) -> state.speedMetersPerSecond).map(Math::abs).map((a) -> averageTranslational - a).map(Math::abs).average().orElse(0);
        // Logger.recordOutput("Drive/Skid Detection/Min Translational Speed", minTranslational);
        // Logger.recordOutput("Drive/Skid Detection/Max Translational Speed", maxTranslational);
        // Logger.recordOutput("Drive/Skid Detection/MaxMin Ratio", maxTranslational / minTranslational);
        // Logger.recordOutput("Drive/Skid Detection/Largest From Average", maxDistanceFromAverage);

        // Clearing log fields
        Logger.recordOutput("Drive/Chassis Speeds/Setpoint", emptySpeeds);
        Logger.recordOutput("Drive/Swerve States/Setpoints", emptyStates);
        Logger.recordOutput("Drive/Swerve States/Setpoints Optimized", emptyStates);
    }

    public void postCommandPeriodic() {
        // if (DriverStation.isDisabled()) {
        //     // TODO: UNCOMMENT IF DRIVE MOVES WITH NO COMMAND AFTER ENABLE
        //     // for (var module : modules) {
        //     //     module.stop();
        //     // }
        // } else
        if (!Objects.equals(translationSubsystem.getCurrentCommand(), rotationalSubsystem.getCurrentCommand())) {
            runSpeeds(setpointSpeeds);
        }
    }

    public void runSetpoints(SwerveModuleState... states) {
        setpointStates = states;
        Logger.recordOutput("Drive/Swerve States/Setpoints", setpointStates);
        IntStream.range(0, modules.length).forEach((i) -> modules[i].runSetpoint(setpointStates[i]));
        Logger.recordOutput("Drive/Swerve States/Setpoints Optimized", setpointStates);
    }

    public void runSpeeds(ChassisSpeeds chassisSpeeds) {
        setpointSpeeds = chassisSpeeds;
        Logger.recordOutput("Drive/Chassis Speeds/Setpoint", setpointSpeeds);
        ChassisSpeeds correctedSpeeds = ChassisSpeeds.discretize(setpointSpeeds, rotationCorrection.get());
        setpointStates = DriveConstants.kinematics.toSwerveModuleStates(correctedSpeeds, centerOfRotation);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DriveConstants.maxDriveSpeed);
        runSetpoints(setpointStates);
    }

    public void driveVelocity(double vx, double vy, double omega) {
        setpointSpeeds.vxMetersPerSecond = vx;
        setpointSpeeds.vyMetersPerSecond = vy;
        setpointSpeeds.omegaRadiansPerSecond = omega;
    }
    public void driveVelocity(ChassisSpeeds speeds) {
        driveVelocity(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
    }

    public void drivePPVelocity(ChassisSpeeds speeds, DriveFeedforwards ff) {
        //TODO: Actually do something with PP ff
        driveVelocity(speeds);
    }

    public void setCenterOfRotation(Translation2d cor) {
        centerOfRotation = cor;
        Logger.recordOutput("Drive/Center of Rotation", getPose().transformBy(new Transform2d(centerOfRotation, Rotation2d.kZero)));
    }

    /** Zeros the drive encoders. */
    // public void zeroEncoders() {
    //     for(var module : modules) {
    //         module.zeroEncoders();
    //     }
    // }

    /** Stops the drive. */
    public void stop() {
        Arrays.stream(modules).forEach(Module::stop);
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement.
     * The modules will
     * return to their normal orientations the next time a nonzero velocity is
     * requested.
     */
    public void stopWithX() {
        IntStream.range(0, DriveConstants.moduleConstants.length).forEach((i) -> {
            setpointStates[i] = new SwerveModuleState(
                0,
                DriveConstants.moduleConstants[i].moduleTranslation.getAngle()
            );
        });
    }

    public void setBrakeMode(boolean enabled) {
        for (var module : modules) {
            module.setBrakeMode(enabled);
        }
    }

    /** Returns the maximum linear speed in meters per sec. */
    // public double getMaxLinearSpeedMetersPerSec() {
    //     return DriveConstants.maxDriveSpeed.in(MetersPerSecond)
    //         * DriveConstants.maxDriveSpeedEnvCoef.getAsDouble()
    //     ;
    // }

    /** Returns the maximum angular speed in radians per sec. */
    // public double getMaxAngularSpeedRadiansPerSec() {
    //     return DriveConstants.maxTurnRate.in(RadiansPerSecond)
    //         * DriveConstants.maxTurnRateEnvCoef.getAsDouble()
    //     ;
    // }

    /**
     * Returns the measured X, Y, and theta field velocities in meters per sec. The
     * components of the
     * twist are velocities and NOT changes in position.
     */
    public Twist2d getFieldVelocity() {
        return fieldVelocity;
    }

    /** Returns the current yaw (Z rotation). */
    public Rotation2d getGyroRotation() {
        return gyroInputs.rotation.toRotation2d();
    }

    /** Returns the current yaw (Z rotation). */
    public Angle getYaw() {
        return gyroInputs.rotation.getMeasureZ();
    }

    /** Returns the current pitch (Y rotation). */
    public Angle getPitch() {
        return gyroInputs.rotation.getMeasureY();
    }

    /** Returns the current roll (X rotation). */
    public Angle getRoll() {
        return gyroInputs.rotation.getMeasureX();
    }

    /** Returns the current pitch velocity (Y rotation) in radians per second. */
    public AngularVelocity getYawVelocity() {
        return gyroInputs.yawVelocity;
    }

    /** Returns the current pitch velocity (Y rotation) in radians per second. */
    public AngularVelocity getPitchVelocity() {
        return gyroInputs.pitchVelocity;
    }

    /** Returns the current roll velocity (X rotation) in radians per second. */
    public AngularVelocity getRollVelocity() {
        return gyroInputs.rollVelocity;
    }

    /** Returns the current odometry pose. */
    public Pose2d getPose() {
        return RobotState.getInstance().getPose();
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    /** Resets the current odometry pose. */
    public void setPose(Pose2d newPose) {
        gyroIO.resetYaw(newPose.getRotation().getMeasure());
        RobotState.getInstance().setPose(getGyroRotation(), getModulePositions(), newPose);
    }

    /** Adds vision data to the pose esimation. */
    // public void addVisionData(List<TimestampedVisionUpdate> visionData) {
    // poseEstimator.addVisionData(visionData);
    // }

    /** Returns an array of module positions. */
    public SwerveModulePosition[] getModulePositions() {
        return Arrays.stream(modules).map(Module::getModulePosition).toArray(SwerveModulePosition[]::new);
    }

    /** Returns an array of module positions. */
    public SwerveModulePosition[] getModulePositionDeltas() {
        return Arrays.stream(modules).map(Module::getModulePositionDelta).toArray(SwerveModulePosition[]::new);
    }

    /** Returns the average drive distance in radians */
    public double getAverageModuleDistance() {
        double avgDist = 0.0;
        for (int i = 0; i < DriveConstants.moduleConstants.length; i++) {
            avgDist += Math.abs(modules[i].getWheelAngularPosition().in(Radians));
        }
        return avgDist / DriveConstants.moduleConstants.length;
    }

    public ChassisSpeeds getRobotMeasuredSpeeds() {
        return robotMeasuredSpeeds;
    }

    public ChassisSpeeds getFieldMeasuredSpeeds() {
        return fieldMeasuredSpeeds;
    }

    /** Returns the average drive velocity in radians/sec. */
    public double getCharacterizationVelocity() {
        return Arrays.stream(modules).map(Module::getWheelAngularVelocity).mapToDouble(AngularVelocity::baseUnitMagnitude).average().orElse(0);
    }

    // public boolean collisionDetected() {
    //     return currentSpikeTimer.hasElapsed(currentSpikeTime.in(Seconds));
    // }

    private static final LoggedTunableNumber tP = new LoggedTunableNumber("AutoDrive/tP", 1);
    private static final LoggedTunableNumber tI = new LoggedTunableNumber("AutoDrive/tI", 0);
    private static final LoggedTunableNumber tD = new LoggedTunableNumber("AutoDrive/tD", 0);
    private static final LoggedTunableNumber rP = new LoggedTunableNumber("AutoDrive/rP", 1.5);
    private static final LoggedTunableNumber rI = new LoggedTunableNumber("AutoDrive/rI", 0);
    private static final LoggedTunableNumber rD = new LoggedTunableNumber("AutoDrive/rD", 0);
    public static PPHolonomicDriveController autoConfig() {
        return new PPHolonomicDriveController(
            new PIDConstants(
                tP.get(),
                tI.get(),
                tD.get()
            ),
            new PIDConstants(
                rP.get(),
                rI.get(),
                rD.get()
            )
        );
    }
    public static RobotConfig robotConfig() {
        return new RobotConfig(Pounds.of(0), KilogramSquareMeters.of(0), new com.pathplanner.lib.config.ModuleConfig(DriveConstants.wheelRadius, DriveConstants.maxDriveSpeed, 0, DCMotor.getFalcon500(1), Amps.of(0), 1), DriveConstants.driveBaseRadius);
    }

    private static final LoggedTunableNumber kAutoDriveMaxVelocity = new LoggedTunableNumber("Drive/kAutoDriveMaxVelocity", 3);
    private static final LoggedTunableNumber kMaxAcceleration = new LoggedTunableNumber("Drive/kMaxAcceleration", 5);
    private static final LoggedTunableNumber kMaxAngularVelocity = new LoggedTunableNumber("Drive/kMaxAngularVelocity", Units.degreesToRadians(540));
    private static final LoggedTunableNumber kMaxAngularAcceleration = new LoggedTunableNumber("Drive/kMaxAngularAcceleration", Units.degreesToRadians(720));
    private static final PathConstraints pathConstraints = new PathConstraints(
        kAutoDriveMaxVelocity.get(),
        kMaxAcceleration.get(),
        kMaxAngularVelocity.get(),
        kMaxAngularAcceleration.get()
    );

    // private static final Map<Pose2d, String> locationNames = new HashMap<>(Map.of(
    //     FieldConstants.amp, "Amp",
    //     FieldConstants.subwooferFront, "Subwoofer Front",
    //     FieldConstants.podium, "Podium",
    //     FieldConstants.pathfindSource, "Source",
    //     FieldConstants.pathfindSpeaker, "Speaker Offset"
    // ));

    // public static final String autoDrivePrefix = "AutoDrive";

    // public Command driveToFlipped(Pose2d pos) {
    //     String name = locationNames.entrySet().stream()
    //         .filter(e -> e.getKey().equals(pos))
    //         .findFirst()
    //         .map(Map.Entry::getValue)
    //         .orElse(pos.toString());

    //     return AutoBuilder.pathfindToPoseFlipped(pos, pathConstraints, 0, 0).withName(String.format("%s (%s)", autoDrivePrefix, name));
    // }

    public final Translational translationSubsystem;
    public static class Translational extends SubsystemBase {
        public final Drive drive;
        
        private Translational(Drive drive) {
            this.drive = drive;
            setName("Drive/Translational");
            SmartDashboard.putData("Subsystems/Drive/Translational", this);
        }

        public void driveVelocity(double vx, double vy) {
            drive.setpointSpeeds.vxMetersPerSecond = vx;
            drive.setpointSpeeds.vyMetersPerSecond = vy;
        }
        public void driveVelocity(ChassisSpeeds speeds) {
            driveVelocity(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        }

        public void stop() {
            driveVelocity(0,0);
        }

        public Command fieldRelative(Supplier<ChassisSpeeds> speeds) {
            var subsystem = this;
            return new Command() {
                {
                    addRequirements(subsystem);
                    setName("Field Relative");
                }
                @Override
                public void execute() {
                    driveVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds.get(), drive.getRotation()));
                }
                @Override
                public void end(boolean interrupted) {
                    stop();
                }
            };
        }

        public static Supplier<ChassisSpeeds> joystickSpectatorToFieldRelative(Joystick translationalJoystick, BooleanSupplier precisionSupplier) {
            return () -> {
                var fieldVec = Perspective.getCurrent().toField(
                    translationalJoystick.toVector()
                    .times(
                        DriveConstants.maxDriveSpeed.in(MetersPerSecond) * 
                        DriveConstants.maxDriveSpeedEnvCoef.getAsDouble() * 
                        (precisionSupplier.getAsBoolean() ? DriveConstants.precisionLinearMultiplier : 1)
                    )
                );
                return new ChassisSpeeds(
                    fieldVec.get(0),
                    fieldVec.get(1),
                    0
                );
            };
        }
    }
    public final Rotational rotationalSubsystem;
    public static class Rotational extends SubsystemBase {
        public final Drive drive;
        
        private Rotational(Drive drive) {
            this.drive = drive;
            setName("Drive/Rotational");
            SmartDashboard.putData("Subsystems/Drive/Rotational", this);
        }

        public void driveVelocity(double omega) {
            drive.setpointSpeeds.omegaRadiansPerSecond = omega;
        }
        public void driveVelocity(ChassisSpeeds speeds) {
            driveVelocity(speeds.omegaRadiansPerSecond);
        }
        public void stop() {
            driveVelocity(0);
        }

        public Command spin(DoubleSupplier omega) {
            return Commands.runEnd(() -> driveVelocity(omega.getAsDouble()), this::stop, this);
        }

        public Command defenseSpin(Joystick joystick) {
            var subsystem = this;
            return new Command() {
                {
                    addRequirements(subsystem);
                    setName("Defense Spin");
                }
                private static final LoggedTunableNumber defenseSpinLinearThreshold = new LoggedTunableNumber("Drive/Defense Spin Linear Threshold", 0.125);
                private static final Matrix<N2, N2> perpendicularMatrix = 
                    MatBuilder.fill(
                        Nat.N2(), Nat.N2(), 
                        +0,-1,
                        +1,+0
                    )
                ;
                @Override
                public void execute() {
                    Leds.getInstance().defenseSpin.setFlag(true);
                    var joyVec = Perspective.getCurrent().toField(joystick.toVector());
                    var desiredLinear = VecBuilder.fill(drive.setpointSpeeds.vxMetersPerSecond, drive.setpointSpeeds.vyMetersPerSecond);
                    var fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(drive.setpointSpeeds, drive.getRotation());
                    var perpendicularLinear = new Vector<N2>(perpendicularMatrix.times(
                        VecBuilder.fill(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond)
                    ));
                    var dot = joystick.x().getAsDouble();
                    if(desiredLinear.norm() > defenseSpinLinearThreshold.get()) {
                        dot = -joyVec.dot(perpendicularLinear);
                    }
                    var omega = dot
                        * DriveConstants.maxTurnRate.in(RadiansPerSecond)
                        * DriveConstants.maxTurnRateEnvCoef.getAsDouble() * 0.25
                    ;
                    driveVelocity(omega);
                    if(desiredLinear.norm() <= defenseSpinLinearThreshold.get()) {
                        drive.setCenterOfRotation(new Translation2d());
                        return;
                    }
                    var rotateAround = MathExtraUtil.vectorFromRotation(
                        MathExtraUtil.rotationFromVector(desiredLinear)
                        // .plus(Rotation2d.fromDegrees(45 * Math.signum(velo)))
                    );
                    drive.setCenterOfRotation(
                        Arrays.stream(DriveConstants.moduleTranslations)
                        .map((t) -> new Translation2d(t.toVector().unit().times(RobotConstants.centerToBumperCorner.in(Meters))))
                        .sorted((a, b) -> 
                            (int) Math.signum(
                                b.toVector().unit().dot(rotateAround) - a.toVector().unit().dot(rotateAround)
                            )    
                        )
                        .findFirst()
                        .orElse(new Translation2d())
                    );
                }
                @Override
                public void end(boolean interrupted) {
                    stop();
                    drive.setCenterOfRotation(new Translation2d());
                    Leds.getInstance().defenseSpin.setFlag(false);
                }
            };
        }

        public Command pidControlledHeading(Supplier<Optional<Rotation2d>> headingSupplier) {
            var subsystem = this;
            return new Command() {
                private final ProfiledPIDController headingPID = new ProfiledPIDController(
                    DriveConstants.headingKp,
                    DriveConstants.headingKi,
                    DriveConstants.headingKd,
                    new Constraints(
                        DriveConstants.maxTurnRate.in(RadiansPerSecond),
                        5000
                    )
                );
                {
                    addRequirements(subsystem);
                    setName("PID Controlled Heading");
                    headingPID.enableContinuousInput(-Math.PI, Math.PI);
                    headingPID.setTolerance(DriveConstants.headingTolerance.in(Radians), DriveConstants.omegaTolerance.in(RadiansPerSecond));
                }
                private Rotation2d desiredHeading;
                private boolean headingSet;
                @Override
                public void initialize() {
                    desiredHeading = drive.getPose().getRotation();
                    headingPID.reset(drive.getRotation().getRadians());
                }
                @Override
                public void execute() {
                    var heading = headingSupplier.get();
                    headingSet = heading.isPresent();
                    heading.ifPresent((r) -> desiredHeading = r);
                    double turnInput = headingPID.calculate(drive.getRotation().getRadians(), desiredHeading.getRadians());
                    turnInput = headingPID.atSetpoint() ? 0 : turnInput + headingPID.getSetpoint().velocity;
                    turnInput = MathUtil.clamp(
                        turnInput, 
                        -0.5 * DriveConstants.maxTurnRateEnvCoef.getAsDouble(), 
                        +0.5 * DriveConstants.maxTurnRateEnvCoef.getAsDouble()
                    );
                    driveVelocity(turnInput * DriveConstants.maxTurnRate.in(RadiansPerSecond));
                }
                @Override
                public void end(boolean interrupted) {
                    stop();
                }
                @Override
                public boolean isFinished() {
                    return !headingSet && headingPID.atSetpoint();
                }
            };
        }

        public Command headingFromJoystick(Joystick joystick, Rotation2d[] snapPoints, Supplier<Rotation2d> forwardDirectionSupplier) {
            return pidControlledHeading(
                new LazyOptional<Rotation2d>() {
                    private final Timer preciseTurnTimer = new Timer();
                    private final double preciseTurnTimeThreshold = 0.5;
                    private Optional<Rotation2d> outputMap(Rotation2d i) {
                        return Optional.of(i.minus(forwardDirectionSupplier.get()));
                    }
                    @Override
                    public Optional<Rotation2d> get() {
                        if(joystick.magnitude() == 0) {
                            preciseTurnTimer.restart();
                            return Optional.empty();
                        }
                        var joyHeading = MathExtraUtil.rotationFromVector(Perspective.getCurrent().toField(joystick.toVector()));
                        if(preciseTurnTimer.hasElapsed(preciseTurnTimeThreshold)) {
                            return outputMap(joyHeading);
                        }
                        int smallestDistanceIndex = 0;
                        double smallestDistance = Double.MAX_VALUE;
                        for(int i = 0; i < snapPoints.length; i++) {
                            var dist = Math.abs(joyHeading.minus(AllianceFlipUtil.apply(snapPoints[i])).getRadians());
                            if(dist < smallestDistance) {
                                smallestDistance = dist;
                                smallestDistanceIndex = i;
                            }
                        }
                        return outputMap(AllianceFlipUtil.apply(snapPoints[smallestDistanceIndex]));
                    }
                }
            );
        }

        public Command pointTo(Supplier<Optional<Translation2d>> posToPointTo, Supplier<Rotation2d> forward) {
            return pidControlledHeading(
                () -> posToPointTo.get().map((pointTo) -> {
                    var FORR = pointTo.minus(RobotState.getInstance().getPose().getTranslation());
                    return new Rotation2d(FORR.getX(), FORR.getY()).minus(forward.get());
                })
            );
        }
    }
}
