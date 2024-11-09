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
import static edu.wpi.first.units.Units.Volts;

import java.util.Arrays;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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
import edu.wpi.first.math.controller.PIDController;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    private final Module[] modules = new Module[DriveConstants.modules.length];

    private SwerveModuleState[] measuredStates = new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
    };
    private ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds();
    private ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds();

    private boolean isCharacterizing = false;
    private final MutVoltage characterizationVolts = Volts.mutable(0);
    private final LoggedTunableNumber rotationCorrection = new LoggedTunableNumber("Drive/Rotation Correction", 0.125);

    private ChassisSpeeds setpointSpeeds = new ChassisSpeeds();
    private Translation2d centerOfRotation = new Translation2d();
    private SwerveModuleState[] setpointStates = new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
    };
    private Timer lastMovementTimer = new Timer(); // used for brake mode

    private Twist2d fieldVelocity = new Twist2d();

    public Drive(GyroIO gyroIO, ModuleIO flModuleIO, ModuleIO frModuleIO, ModuleIO blModuleIO, ModuleIO brModuleIO) {
        System.out.println("[Init Drive] Instantiating Drive");
        this.gyroIO = gyroIO;
        System.out.println("[Init Drive] Gyro IO: " + this.gyroIO.getClass().getSimpleName());
        ModuleIO[] moduleIOs = new ModuleIO[]{flModuleIO, frModuleIO, blModuleIO, brModuleIO};
        for(int i = 0; i < DriveConstants.modules.length; i++) {
            ModuleConstants config = DriveConstants.modules[i];
            System.out.println("[Init Drive] Instantiating Module " + config.name + " with Module IO: " + moduleIOs[i].getClass().getSimpleName());
            var module = new Module(moduleIOs[i], config);
            module.setBrakeMode(false);
            modules[i] = module;
        }
        lastMovementTimer.start();

        // initialize pose estimator
        Pose2d initialPoseMeters = new Pose2d();
        RobotState.getInstance().initializePoseEstimator(DriveConstants.kinematics, getGyroRotation(), getModulePositions(), initialPoseMeters);
        gyroAngle = getPose().getRotation();

        this.translationSubsystem = new Translational(this);
        this.rotationalSubsystem = new Rotational(this);
        if (!AutoBuilder.isConfigured()) {
            AutoBuilder.configure(
                this::getPose,
                this::setPose,
                this::getRobotRelativeSpeeds,
                this::driveVelocity,
                autoConfig(),
                robotConfig(),
                AllianceFlipUtil::shouldFlip,
                translationSubsystem,
                rotationalSubsystem
            );
        }
    }

    public void periodic() {
        // update IO inputs
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Inputs/Drive/Gyro", gyroInputs);
        Arrays.stream(modules).forEach(Module::periodic);

        measuredStates = Arrays.stream(modules).map(Module::getState).toArray(SwerveModuleState[]::new);
        Logger.recordOutput("Drive/SwerveStates/Measured", measuredStates);

        // Update odometry
        // Update field velocity
        robotRelativeSpeeds = DriveConstants.kinematics.toChassisSpeeds(measuredStates);
        if (gyroInputs.connected) {
            gyroAngle = getGyroRotation();
            robotRelativeSpeeds.omegaRadiansPerSecond = gyroInputs.yawVelocity.in(RadiansPerSecond);
        } else {
            // either the gyro is disconnected or we are in a simulation
            // accumulate a gyro estimate using module kinematics
            var wheelDeltas = getModulePositionDeltas(); // get change in module positions
            Twist2d twist = DriveConstants.kinematics.toTwist2d(wheelDeltas); // dtheta will be the estimated change in chassis angle
            gyroAngle = gyroAngle.plus(Rotation2d.fromRadians(twist.dtheta));
        }
        Logger.recordOutput("Drive/Chassis Speeds/Measured", robotRelativeSpeeds);
        RobotState.getInstance().addDriveMeasurement(gyroAngle, getModulePositions());
        fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, gyroAngle);

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
    }

    public void postCommandPeriodic() {
        ChassisSpeeds correctedSpeeds = ChassisSpeeds.discretize(setpointSpeeds, rotationCorrection.get());
        setpointStates = DriveConstants.kinematics.toSwerveModuleStates(correctedSpeeds, centerOfRotation);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DriveConstants.maxDriveSpeed);

        Logger.recordOutput("Drive/Chassis Speeds/Setpoint", setpointSpeeds);
        var setpointStatesOptimized = new SwerveModuleState[setpointStates.length];
        // Run modules
        if (DriverStation.isDisabled()) {
            // Stop moving while disabled
            // TODO: UNCOMMENT IF DRIVE MOVES WITH NO COMMAND AFTER ENABLE
            // for (var module : modules) {
            //     module.stop();
            // }
        } else if (isCharacterizing) {
            for (int i = 0; i < DriveConstants.modules.length; i++) {
                modules[i].runVoltage(characterizationVolts, setpointStates[i].angle);
            }
        } else {
            // Set to last angles if zero
            // if (MathExtraUtil.isNear(new ChassisSpeeds(), correctedSpeeds, 0.05, DriveConstants.headingTolerance.in(Radians))) {
            //     for (int i = 0; i < DriveConstants.modules.length; i++) {
            //         setpointStates[i] = new SwerveModuleState(0.0, setpointStates[i].angle);
            //     }
            // }

            // Send setpoints to modules
            setpointStatesOptimized = new SwerveModuleState[DriveConstants.modules.length];
            for (int i = 0; i < DriveConstants.modules.length; i++) {
                setpointStatesOptimized[i] = modules[i].runSetpoint(setpointStates[i]);
            }
        }
        Logger.recordOutput("Drive/SwerveStates/Setpoints", setpointStates);
        // Logger.recordOutput("Drive/SwerveStates/SetpointsOptimized", setpointStatesOptimized);
        Logger.recordOutput("Drive/Center of Rotation", getPose().transformBy(new Transform2d(centerOfRotation, new Rotation2d())));
    }

    public final Translational translationSubsystem;
    public static class Translational extends SubsystemBase {
        public final Drive drive;
        
        private Translational(Drive drive) {
            this.drive = drive;
            setName("Drive/Translational");
            SmartDashboard.putData("Subsystems/Drive/Translational", this);
        }

        public void driveVelocity(ChassisSpeeds speeds) {
            drive.setpointSpeeds.vxMetersPerSecond = speeds.vxMetersPerSecond;
            drive.setpointSpeeds.vyMetersPerSecond = speeds.vyMetersPerSecond;
        }

        public void stop() {
            driveVelocity(new ChassisSpeeds());
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
                    Leds.getInstance().defenseSpin.setFlag(true);
                }
            };
        }

        public Command pidControlledHeading(Supplier<Optional<Rotation2d>> headingSupplier) {
            var subsystem = this;
            return new Command() {
                private final PIDController headingPID = new PIDController(DriveConstants.headingKp, DriveConstants.headingKi, DriveConstants.headingKd);
                {
                    addRequirements(subsystem);
                    setName("PID Controlled Heading");
                    headingPID.enableContinuousInput(-Math.PI, Math.PI);  // since gyro angle is not limited to [-pi, pi]
                    headingPID.setTolerance(DriveConstants.headingTolerance.in(Radians), DriveConstants.omegaTolerance.in(RadiansPerSecond));
                }
                private Rotation2d desiredHeading;
                private boolean headingSet;
                @Override
                public void initialize() {
                    desiredHeading = drive.getPose().getRotation();
                }
                @Override
                public void execute() {
                    var heading = headingSupplier.get();
                    headingSet = heading.isPresent();
                    heading.ifPresent((r) -> desiredHeading = r);
                    double turnInput = headingPID.calculate(drive.getRotation().getRadians(), desiredHeading.getRadians());
                    turnInput = headingPID.atSetpoint() ? 0 : turnInput;
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

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void driveVelocity(ChassisSpeeds speeds) {
        // isCharacterizing = false;
        setpointSpeeds = speeds;
        // speeds will be applied next drive.periodic()
    }

    public void drivePPVelocity(ChassisSpeeds speeds, DriveFeedforwards ff) {
        //TODO: Actually do something with PP ff
        driveVelocity(speeds);
    }

    public void drivePercent(ChassisSpeeds speeds) {
        driveVelocity(new ChassisSpeeds(
            speeds.vxMetersPerSecond * DriveConstants.maxDriveSpeed.in(MetersPerSecond)
                * DriveConstants.maxDriveSpeedEnvCoef.getAsDouble()
            ,
            speeds.vyMetersPerSecond * DriveConstants.maxDriveSpeed.in(MetersPerSecond)
                * DriveConstants.maxDriveSpeedEnvCoef.getAsDouble()
            ,
            speeds.omegaRadiansPerSecond * DriveConstants.maxTurnRate.in(RadiansPerSecond)
                * DriveConstants.maxTurnRateEnvCoef.getAsDouble()
        ));
    }

    public void setCenterOfRotation(Translation2d cor) {
        centerOfRotation = cor;
    }

    /** Zeros the drive encoders. */
    public void zeroEncoders() {
        for(var module : modules) {
            module.zeroEncoders();
        }
    }

    /** Stops the drive. */
    public void stop() {
        driveVelocity(new ChassisSpeeds());
    }

    public void setBrakeMode(Boolean enabled) {
        for (var module : modules) {
            module.setBrakeMode(enabled);
        }
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement.
     * The modules will
     * return to their normal orientations the next time a nonzero velocity is
     * requested.
     */
    public void stopWithX() {
        stop();
        for (int i = 0; i < DriveConstants.modules.length; i++) {
            setpointStates[i] = new SwerveModuleState(
                setpointStates[i].speedMetersPerSecond, DriveConstants.modules[i].moduleTranslation.getAngle()
            );
        }
    }

    /** Returns the maximum linear speed in meters per sec. */
    public double getMaxLinearSpeedMetersPerSec() {
        return DriveConstants.maxDriveSpeed.in(MetersPerSecond)
            * DriveConstants.maxDriveSpeedEnvCoef.getAsDouble()
        ;
    }

    /** Returns the maximum angular speed in radians per sec. */
    public double getMaxAngularSpeedRadiansPerSec() {
        return DriveConstants.maxTurnRate.in(RadiansPerSecond)
            * DriveConstants.maxTurnRateEnvCoef.getAsDouble()
        ;
    }

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
        return getYaw();
    }

    /** Returns the current yaw (Z rotation). */
    public Rotation2d getYaw() {
        return new Rotation2d(gyroInputs.rotation.getMeasureZ());
    }

    /** Returns the current pitch (Y rotation). */
    public Rotation2d getPitch() {
        return new Rotation2d(gyroInputs.rotation.getMeasureY());
    }

    /** Returns the current roll (X rotation). */
    public Rotation2d getRoll() {
        return new Rotation2d(gyroInputs.rotation.getMeasureX());
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
        RobotState.getInstance().setPose(getGyroRotation(), getModulePositions(), newPose);
    }

    /** Adds vision data to the pose esimation. */
    // public void addVisionData(List<TimestampedVisionUpdate> visionData) {
    // poseEstimator.addVisionData(visionData);
    // }

    /** Returns an array of module positions. */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[DriveConstants.modules.length];
        for (int i = 0; i < DriveConstants.modules.length; i++) {
            modulePositions[i] = modules[i].getPosition();
        }
        return modulePositions;
    }

    /** Returns an array of module positions. */
    public SwerveModulePosition[] getModulePositionDeltas() {
        SwerveModulePosition[] modulePositionDeltas = new SwerveModulePosition[DriveConstants.modules.length];
        for (int i = 0; i < DriveConstants.modules.length; i++) {
            modulePositionDeltas[i] = modules[i].getPositionDelta();
        }
        return modulePositionDeltas;
    }

    /** Returns the average drive distance in radians */
    public double getAverageModuleDistance() {
        double avgDist = 0.0;
        for (int i = 0; i < DriveConstants.modules.length; i++) {
            avgDist += Math.abs(modules[i].getAngularPosition().in(Radians));
        }
        return avgDist / DriveConstants.modules.length;
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return robotRelativeSpeeds;
    }

    public ChassisSpeeds getFieldRelativeSpeeds() {
        return fieldRelativeSpeeds;
    }

    /** Runs forwards at the commanded voltage. */
    public void runCharacterizationVolts(Voltage volts) {
        isCharacterizing = true;
        characterizationVolts.mut_replace(volts);
    }

    /** Returns the average drive velocity in radians/sec. */
    public double getCharacterizationVelocity() {
        double driveVelocityAverage = 0.0;
        for (int i = 0; i < DriveConstants.modules.length; i++) {
            driveVelocityAverage += modules[i].getCharacterizationVelocity();
        }
        return driveVelocityAverage / DriveConstants.modules.length;
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
}
