// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.DriveModulePosition;
import frc.robot.Constants.RobotConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.commands.FieldOrientedDrive.SpectatorType;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.AllianceFlipUtil.FieldFlipType;
import frc.robot.util.LazyOptional;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.MathExtraUtil;
import frc.robot.util.VirtualSubsystem;
import frc.robot.util.controllers.Joystick;
import frc.robot.util.pathplanner.AutoBuilder;

public class Drive extends VirtualSubsystem {
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private Rotation2d prevGyroYaw = new Rotation2d();

    private final Module[] modules = new Module[DriveConstants.numDriveModules]; // FL, FR, BL, BR

    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(DriveModulePosition.moduleTranslations);
    private SwerveModuleState[] lastMeasuredStates = new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
    };

    private boolean isCharacterizing = false;
    private double characterizationVolts = 0.0;
    private final LoggedTunableNumber rotationCorrection = new LoggedTunableNumber("Drive/Rotation Correction", 0.125);

    private ChassisSpeeds setpoint = new ChassisSpeeds();
    private Translation2d centerOfRotation = new Translation2d();
    private SwerveModuleState[] lastSetpointStates = new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
    };
    private Timer lastMovementTimer = new Timer(); // used for brake mode

    private Twist2d fieldVelocity = new Twist2d();

    private final Timer currentSpikeTimer = new Timer();
    private static final LoggedTunableNumber currentSpikeThreshold = new LoggedTunableNumber("Drive/Current Spike Threshold", 0); 
    private static final LoggedTunableNumber currentSpikeTime = new LoggedTunableNumber("Drive/Current Spike Time", 0);
    private final ArrayList<Module> spikeModules = new ArrayList<>();

    public Drive(GyroIO gyroIO, ModuleIO flModuleIO, ModuleIO frModuleIO, ModuleIO blModuleIO, ModuleIO brModuleIO) {
        System.out.println("[Init Drive] Instantiating Drive");
        this.gyroIO = gyroIO;
        System.out.println("[Init Drive] Gyro IO: " + this.gyroIO.getClass().getSimpleName());
        // SmartDashboard.putData("Subsystems/Drive", this);
        ModuleIO[] moduleIOs = new ModuleIO[]{flModuleIO, frModuleIO, blModuleIO, brModuleIO};
        for(DriveModulePosition position : DriveModulePosition.values()) {
            System.out.println("[Init Drive] Instantiating Module " + position.name() + " with Module IO: " + moduleIOs[position.ordinal()].getClass().getSimpleName());
            modules[position.ordinal()] = new Module(moduleIOs[position.ordinal()], position);
        }
        lastMovementTimer.start();
        for (var module : modules) {
            module.setBrakeMode(false);
        }

        // initialize pose estimator
        Pose2d initialPoseMeters = new Pose2d();
        RobotState.getInstance().initializePoseEstimator(kinematics, getGyroRotation(), getModulePositions(), initialPoseMeters);
        prevGyroYaw = getPose().getRotation();

        this.translationSubsystem = new Translational(this);
        this.rotationalSubsystem = new Rotational(this);

        if (!AutoBuilder.isConfigured()) {
            AutoBuilder.configureHolonomic(
                this::getPose,
                this::setPose,
                this::getChassisSpeeds,
                this::driveVelocity,
                autoConfigSup.get(),
                AllianceFlipUtil::shouldFlip,
                translationSubsystem,
                rotationalSubsystem
            );
        }
    }

    public void periodic() {
        // update IO inputs
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        for (var module : modules) {
            module.periodic();
        }

        for (var module : modules) {
            if (Math.abs(module.getCurrentAmps()) >= currentSpikeThreshold.get()) {
                if (!spikeModules.contains(module)) {
                    spikeModules.add(module);
                }
            } else {
                spikeModules.remove(module);
            }
        }

        if (!spikeModules.isEmpty()) {
            currentSpikeTimer.start();
        } else {
            currentSpikeTimer.stop();
            currentSpikeTimer.reset();
        }

        // Run modules
        if (DriverStation.isDisabled()) {
            // Stop moving while disabled
            for (var module : modules) {
                module.stop();
            }

            // Clear setpoint logs
            Logger.recordOutput("Drive/SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("Drive/SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});

        } else if (isCharacterizing) {
            // Run in characterization mode
            for (var module : modules) {
                module.runCharacterization(characterizationVolts);
            }

            // Clear setpoint logs
            Logger.recordOutput("Drive/SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("Drive/SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});

        } else {
            ChassisSpeeds correctedSpeeds = ChassisSpeeds.discretize(setpoint, rotationCorrection.get());
            SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(correctedSpeeds, centerOfRotation);
            SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DriveConstants.maxDriveSpeedMetersPerSec);

            // Set to last angles if zero
            if (MathExtraUtil.isNear(new ChassisSpeeds(), correctedSpeeds, 0.05, DriveConstants.headingTolerance)) {
                for (int i = 0; i < DriveConstants.numDriveModules; i++) {
                    setpointStates[i] = new SwerveModuleState(0.0, lastSetpointStates[i].angle);
                }
            }
            lastSetpointStates = setpointStates;

            // Send setpoints to modules
            SwerveModuleState[] optimizedStates = new SwerveModuleState[DriveConstants.numDriveModules];
            for (int i = 0; i < DriveConstants.numDriveModules; i++) {
                optimizedStates[i] = modules[i].runSetpoint(setpointStates[i]);
            }

            // Log setpoint states
            Logger.recordOutput("Drive/SwerveStates/Setpoints", setpointStates);
            Logger.recordOutput("Drive/SwerveStates/SetpointsOptimized", optimizedStates);
        }

        // Log measured states
        SwerveModuleState[] measuredStates = new SwerveModuleState[DriveConstants.numDriveModules];
        for (int i = 0; i < DriveConstants.numDriveModules; i++) {
            measuredStates[i] = modules[i].getState();
        }
        Logger.recordOutput("Drive/SwerveStates/Measured", measuredStates);
        lastMeasuredStates = measuredStates;

        // Update odometry
        Rotation2d gyroAngle;
        if (gyroInputs.connected) {
            gyroAngle = getGyroRotation();
        } else {
            // either the gyro is disconnected or we are in a simulation
            // accumulate a gyro estimate using module kinematics
            var wheelDeltas = getModulePositionDeltas(); // get change in module positions
            Twist2d twist = kinematics.toTwist2d(wheelDeltas); // dtheta will be the estimated change in chassis angle
            gyroAngle = prevGyroYaw.plus(Rotation2d.fromRadians(twist.dtheta));
        }
        RobotState.getInstance().addDriveMeasurement(gyroAngle, getModulePositions());

        // Logger.recordOutput("Odometry/Robot", getPose());

        // Update field velocity
        ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(measuredStates);
        Translation2d linearFieldVelocity = new Translation2d(chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond)
                .rotateBy(getRotation());
        fieldVelocity = new Twist2d(
                linearFieldVelocity.getX(),
                linearFieldVelocity.getY(),
                gyroInputs.connected
                        ? gyroInputs.yawVelocityRadPerSec
                        : chassisSpeeds.omegaRadiansPerSecond);

        // Update brake mode
        // for (var module : modules) {
        // module.setBrakeMode(true);
        // }

        // save values for next loop
        prevGyroYaw = gyroAngle;

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
            drive.setpoint.vxMetersPerSecond = speeds.vxMetersPerSecond;
            drive.setpoint.vyMetersPerSecond = speeds.vyMetersPerSecond;
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
                var fieldVec = SpectatorType.getCurrentType().toField(
                    translationalJoystick.toVector()
                    .times(
                        DriveConstants.maxDriveSpeedMetersPerSec * 
                        (precisionSupplier.getAsBoolean() ? DriveConstants.precisionLinearMultiplier : 1)
                    )
                );
                return AllianceFlipUtil.applyFieldRelative(
                    new ChassisSpeeds(
                        fieldVec.get(0),
                        fieldVec.get(1),
                        0
                    ),
                    FieldFlipType.CenterPointFlip
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
            drive.setpoint.omegaRadiansPerSecond = omega;
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

        private final LoggedTunableNumber defenseSpinLinearThreshold = new LoggedTunableNumber("Drive/Defense Spin Linear Threshold", 0.125);

        public Command defenseSpin(Joystick joystick) {
            var subsystem = this;
            return new Command() {
                {
                    addRequirements(subsystem);
                    setName("Defense Spin");
                }
                @Override
                public void initialize() {
                    
                }
                private static final Matrix<N2, N2> perpendicularMatrix = 
                    MatBuilder.fill(
                        Nat.N2(), Nat.N2(), 
                        +0,-1,
                        +1,+0
                    )
                ;
                @Override
                public void execute() {
                    var joyVec = SpectatorType.getCurrentType().toField(joystick.toVector());
                    var desiredLinear = VecBuilder.fill(drive.setpoint.vxMetersPerSecond, drive.setpoint.vyMetersPerSecond);
                    var fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(drive.setpoint, drive.getRotation());
                    var perpendicularLinear = new Vector<N2>(perpendicularMatrix.times(
                        VecBuilder.fill(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond)
                    ));
                    var dot = joystick.x().getAsDouble();
                    if(desiredLinear.norm() > defenseSpinLinearThreshold.get()) {
                        dot = -joyVec.dot(perpendicularLinear);
                    }
                    var omega = dot * DriveConstants.maxTurnRateRadiansPerSec * 0.25;
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
                        Arrays.stream(DriveConstants.DriveModulePosition.moduleTranslations)
                        .map((t) -> new Translation2d(t.toVector().unit().times(RobotConstants.centerToBumperCornerMeters)))
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
                    headingPID.setTolerance(DriveConstants.headingTolerance);
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
                    turnInput = MathUtil.clamp(turnInput, -0.5, +0.5);
                    driveVelocity(turnInput * DriveConstants.maxTurnRateRadiansPerSec);
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
                        var joyVec = SpectatorType.getCurrentType().toField(joystick.toVector());
                        Rotation2d joyHeading = AllianceFlipUtil.apply(MathExtraUtil.rotationFromVector(joyVec), FieldFlipType.CenterPointFlip);
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
        isCharacterizing = false;
        setpoint = speeds;
        // speeds will be applied next drive.periodic()
    }

    public void drivePercent(ChassisSpeeds speeds) {
        driveVelocity(new ChassisSpeeds(
                speeds.vxMetersPerSecond * DriveConstants.maxDriveSpeedMetersPerSec,
                speeds.vyMetersPerSecond * DriveConstants.maxDriveSpeedMetersPerSec,
                speeds.omegaRadiansPerSecond * DriveConstants.maxTurnRateRadiansPerSec));
    }

    public void setCenterOfRotation(Translation2d cor) {
        centerOfRotation = cor;
    }

    /** Zeros the drive encoders. */
    public void zeroEncoders() {
        for (int i = 0; i < DriveConstants.numDriveModules; i++) {
            modules[i].zeroEncoders();
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
        for (int i = 0; i < DriveConstants.numDriveModules; i++) {
            lastSetpointStates[i] = new SwerveModuleState(
                    lastSetpointStates[i].speedMetersPerSecond, DriveModulePosition.moduleTranslations[i].getAngle());
        }
    }

    /** Returns the maximum linear speed in meters per sec. */
    public double getMaxLinearSpeedMetersPerSec() {
        return DriveConstants.maxDriveSpeedMetersPerSec;
    }

    /** Returns the maximum angular speed in radians per sec. */
    public double getMaxAngularSpeedRadiansPerSec() {
        return DriveConstants.maxTurnRateRadiansPerSec;
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
        return new Rotation2d(gyroInputs.yawPositionRad);
    }

    /** Returns the current pitch (Y rotation). */
    public Rotation2d getPitch() {
        return new Rotation2d(gyroInputs.pitchPositionRad);
    }

    /** Returns the current roll (X rotation). */
    public Rotation2d getRoll() {
        return new Rotation2d(gyroInputs.rollPositionRad);
    }

    /** Returns the current pitch velocity (Y rotation) in radians per second. */
    public double getPitchVelocity() {
        return gyroInputs.pitchVelocityRadPerSec;
    }

    /** Returns the current roll velocity (X rotation) in radians per second. */
    public double getRollVelocity() {
        return gyroInputs.rollVelocityRadPerSec;
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
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[DriveConstants.numDriveModules];
        for (int i = 0; i < DriveConstants.numDriveModules; i++) {
            modulePositions[i] = modules[i].getPosition();
        }
        return modulePositions;
    }

    /** Returns an array of module positions. */
    public SwerveModulePosition[] getModulePositionDeltas() {
        SwerveModulePosition[] modulePositionDeltas = new SwerveModulePosition[DriveConstants.numDriveModules];
        for (int i = 0; i < DriveConstants.numDriveModules; i++) {
            modulePositionDeltas[i] = modules[i].getPositionDelta();
        }
        return modulePositionDeltas;
    }

    /** Returns the average drive distance in radians */
    public double getAverageModuleDistance() {
        double avgDist = 0.0;
        for (int i = 0; i < DriveConstants.numDriveModules; i++) {
            avgDist += Math.abs(modules[i].getPositionRadians());
        }
        return avgDist / DriveConstants.numDriveModules;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(lastMeasuredStates);
    }

    /** Runs forwards at the commanded voltage. */
    public void runCharacterizationVolts(double volts) {
        isCharacterizing = true;
        characterizationVolts = volts;
    }

    /** Returns the average drive velocity in radians/sec. */
    public double getCharacterizationVelocity() {
        double driveVelocityAverage = 0.0;
        for (int i = 0; i < DriveConstants.numDriveModules; i++) {
            driveVelocityAverage += modules[i].getCharacterizationVelocity();
        }
        return driveVelocityAverage / DriveConstants.numDriveModules;
    }

    public boolean collisionDetected() {
        return currentSpikeTimer.hasElapsed(currentSpikeTime.get());
    }

    private static final LoggedTunableNumber tP = new LoggedTunableNumber("AutoDrive/tP", 1);
    private static final LoggedTunableNumber tI = new LoggedTunableNumber("AutoDrive/tI", 0);
    private static final LoggedTunableNumber tD = new LoggedTunableNumber("AutoDrive/tD", 0);
    private static final LoggedTunableNumber rP = new LoggedTunableNumber("AutoDrive/rP", 1.5);
    private static final LoggedTunableNumber rI = new LoggedTunableNumber("AutoDrive/rI", 0);
    private static final LoggedTunableNumber rD = new LoggedTunableNumber("AutoDrive/rD", 0);
    public static final Supplier<HolonomicPathFollowerConfig> autoConfigSup = () -> {
        return new HolonomicPathFollowerConfig(
            new PIDConstants(
                tP.get(),
                tI.get(),
                tD.get()
            ),
            new PIDConstants(
                rP.get(),
                rI.get(),
                rD.get()
            ),
            DriveConstants.maxDriveSpeedMetersPerSec,
            DriveConstants.driveBaseRadius,
            new ReplanningConfig()
        );
    };

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

    private static final Map<Pose2d, String> locationNames = new HashMap<>(Map.of());

    public static final String autoDrivePrefix = "AutoDrive";

    public Command driveToFlipped(Pose2d pos) {
        String name = locationNames.entrySet().stream()
            .filter(e -> e.getKey().equals(pos))
            .findFirst()
            .map(Map.Entry::getValue)
            .orElse(pos.toString());

        return AutoBuilder.pathfindToPoseFlipped(pos, pathConstraints, 0, 0).withName(String.format("%s (%s)", autoDrivePrefix, name));
    }
}
