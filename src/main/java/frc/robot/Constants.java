// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import java.util.Arrays;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.util.function.BooleanConsumer;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.vision.apriltag.ApriltagCamera;
import frc.robot.subsystems.vision.apriltag.ApriltagCameraIO;
import frc.robot.util.AllianceFlipUtil.FlippedPose2d;
import frc.robot.util.AllianceFlipUtil.FlippedTranslation3d;
import frc.robot.util.Environment;
import frc.robot.util.GearRatio;
import frc.robot.util.GearRatio.Wheel;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.MathExtraUtil;

public final class Constants {

    public static enum Mode {
        REAL, SIM, REPLAY
    }

    public static final Mode simulationMode = Mode.SIM;
    public static final boolean tuningMode = true;

    public static final double dtSeconds = 0.02;
    public static final double sensorDtSeconds = 0.005;
    public static final double loopFrequencyHz = 1.0/dtSeconds;

    public static Mode getMode() {
        if(Robot.isReal()) return Mode.REAL;
        return (simulationMode == Mode.REAL ? Mode.SIM : simulationMode);
    }

    public static final class CANDevices {

        // Drive
        public static final String driveCanBusName = "rio";
        // | Front Left
        public static final int frontLeftDriveMotorID  = 1;
        public static final int frontLeftTurnMotorID   = 1;
        // | Front Right
        public static final int frontRightDriveMotorID  = 2;
        public static final int frontRightTurnMotorID   = 2;
        // | Back Left
        public static final int backLeftDriveMotorID  = 3;
        public static final int backLeftTurnMotorID   = 3;
        // | Back Right
        public static final int backRightDriveMotorID  = 4;
        public static final int backRightTurnMotorID   = 4;

        // Intake
        public static final int intakeBeltMotorID   = 5;
        public static final int intakeRollerMotorID = 5;

        // Pivot
        public static final int pivotLeftMotorID = 6;
        public static final int pivotRightMotorID = 7;
        public static final int pivotEncoderID = 6;

        // Kicker
        public static final int kickerLeftID = 8;
        public static final int kickerRightID = 9;

        // Shooter
        public static final int shooterLeftID = 8;
        public static final int shooterRightID = 9;
        public static final int shooterAmpID = 6;

        // Climber
        public static final int climberID = 10;

        // Misc
        public static final int pigeonCanID = 0;
        public static final int candleCanID = 0;

        public static final double minCanUpdateRate = 4.0;
    }

    public static final class DIOPorts {
        // HID
        public static final int redButtonPort = 2;
        public static final int whiteButtonPort = 3;
        public static final int yellowButtonPort = 4;

        // Intake
        public static final int intakeSensorPort = 0;

        // Kicker
        public static final int kickerSensorPort = 1;

        // Pivot
        public static final int pivotLeftLimitSwitchPort = 8;
        public static final int pivotRightLimitSwitchPort = 9;
    }

    public static final class RobotConstants {
        public static final Rotation2d shooterForward = Rotation2d.fromDegrees(0);
        public static final Rotation2d intakeForward = Rotation2d.fromDegrees(180);

        /**Distance between the front and back wheels*/
        public static final Distance trackWidthX = Inches.of(25.5);
        /**Distance between the left and right wheels*/
        public static final Distance trackWidthY = Inches.of(25.5);

        /**Distance between back bumper and front bumper, aka in the X axis */
        public static final Distance robotLength = Centimeters.of(90);
        /**Distance between left bumper and right bumper, aka in the Y axis */
        public static final Distance robotWidth = Centimeters.of(90);

        public static final Distance centerToBumperCorner = Meters.of(new Translation2d(robotLength, robotWidth).getNorm());
    }

    public static final class DriveConstants {
        public static class ModuleConfig {
            public final String name;
            public final int driveMotorID;
            public final int turnMotorID;
            // motor direction to drive 'forward' (cancoders at angles given in cancoderOffsetRotations)
            public final InvertedValue driveInverted;
            // absolute position of cancoder when drive wheel is facing 'forward'
            public final Angle cancoderOffset;
            public final Translation2d moduleTranslation;
            public final Vector<N2> positiveRotVec;
            ModuleConfig(String name, int driveMotorID, int turnMotorID, InvertedValue driveInverted, Angle cancoderOffset, Translation2d moduleTranslation) {
                this.name = name;
                this.driveMotorID = driveMotorID;
                this.turnMotorID = turnMotorID;
                this.driveInverted = driveInverted;
                this.cancoderOffset = cancoderOffset;
                this.moduleTranslation = moduleTranslation;
                this.positiveRotVec = MathExtraUtil.vectorFromRotation(this.moduleTranslation.getAngle().plus(Rotation2d.fromDegrees(90)));
            }
        }

        public static final ModuleConfig[] modules = {
            new ModuleConfig(
                "Front Left",
                CANDevices.frontLeftDriveMotorID, CANDevices.frontLeftTurnMotorID,
                InvertedValue.CounterClockwise_Positive,
                Rotations.of(0.75),
                new Translation2d(
                    RobotConstants.trackWidthX.divide(+2),
                    RobotConstants.trackWidthY.divide(+2)
                )
            ),
            new ModuleConfig(
                "Front Right",
                CANDevices.frontRightDriveMotorID, CANDevices.frontRightTurnMotorID,
                InvertedValue.Clockwise_Positive,
                Rotations.of(0.5),
                new Translation2d(
                    RobotConstants.trackWidthX.divide(+2),
                    RobotConstants.trackWidthY.divide(-2)
                )
            ),
            new ModuleConfig(
                "Back Left",
                CANDevices.backLeftDriveMotorID, CANDevices.backLeftTurnMotorID,
                InvertedValue.CounterClockwise_Positive,
                Rotations.of(0.5),
                new Translation2d(
                    RobotConstants.trackWidthX.divide(-2),
                    RobotConstants.trackWidthY.divide(+2)
                )
            ),
            new ModuleConfig(
                "Back Right",
                CANDevices.backRightDriveMotorID, CANDevices.backRightTurnMotorID,
                InvertedValue.Clockwise_Positive,
                Rotations.of(0.75),
                new Translation2d(
                    RobotConstants.trackWidthX.divide(-2),
                    RobotConstants.trackWidthY.divide(-2)
                )
            ),
        };
        public static final Translation2d[] moduleTranslations = Arrays.stream(modules).map((a) -> a.moduleTranslation).toArray(Translation2d[]::new);

        /**Weight with battery and bumpers*/
        public static final double weightKg = Pounds.of(58.0).in(Kilograms);
        
        public static final Distance driveBaseRadius = Meters.of(Arrays.stream(moduleTranslations).mapToDouble((t) -> t.getNorm()).max().orElse(0.5));
        private static final double correctionVal = 314.0 / 320.55;
        public static final Distance wheelRadius = Inches.of(1.5 * correctionVal);

        public static final GearRatio driveWheelGearRatio = new GearRatio()
            .gear(14).gear(22).axle()
            .gear(15).gear(45).axle()
        ;
        public static final Wheel driveWheel = Wheel.radius(wheelRadius);
        public static final GearRatio turnWheelGearRatio = new GearRatio()
            .gear(15).gear(32).axle()
            .gear(10).gear(60).axle()
        ;
        public static final double driveWheelGearReduction = 1.0 / (1.0/4.0);
        public static final double turnWheelGearReduction = 1.0 / ((15.0/32.0)*(10.0/60.0));

        public static final double[] driveRealKps = {0.7, 0.4, 0.7, 0.7};
        public static final double[] driveRealKds = {3.5, 2.5, 3.7, 3.5};

        public static final double driveSnapKp = 1.5;
        public static final double driveSnapKi = 0;
        public static final double driveSnapKd = 0;


        public static final LinearVelocity maxDriveSpeed = MetersPerSecond.of(6);
        /**Tangential speed (m/s) = radial speed (rad/s) * radius (m)*/
        public static final AngularVelocity maxTurnRate = RadiansPerSecond.of(maxDriveSpeed.in(MetersPerSecond) / new Translation2d(RobotConstants.trackWidthX.divide(2), RobotConstants.trackWidthY.divide(2)).getNorm());
        public static final DoubleSupplier maxDriveSpeedEnvCoef = Environment.switchVar(
            () -> 1,
            new LoggedTunableNumber("Demo Constraints/Max Translational Percentage", 0.25)
        );
        public static final DoubleSupplier maxTurnRateEnvCoef = Environment.switchVar(
            () -> 1,
            new LoggedTunableNumber("Demo Constraints/Max Rotational Percentage", 0.25)
        );
        /**full speed in 0.25 sec*/
        public static final double joystickSlewRateLimit = 1.0 / 0.25;
        public static final double driveJoystickDeadbandPercent = 0.2;
        public static final double driveMaxJerk = 200.0;

        public static final double precisionLinearMultiplier = 0.2;
        public static final double precisionTurnMulitiplier = 0;//0.2;

        public static final double poseMoveTranslationkP = 1;
        public static final double poseMoveTranslationMaxVel = 3;
        public static final double poseMoveTranslationMaxAccel = 3;

        public static final double poseMoveRotationkP = 0.05;
        public static final double poseMoveRotationMaxVel = Math.PI;
        public static final double poseMoveRotationMaxAccel = Math.PI;

        public static final double headingKp = 0.2;
        public static final double headingKi = 0;
        public static final double headingKd = 0;
        public static final double headingTolerance = Degrees.of(1).in(Radians);
        public static final double omegaTolerance = DegreesPerSecond.of(1).in(RadiansPerSecond);
    }

    public static final class VisionConstants {
        public static enum Camera {
            LeftApriltag(
                "Left Apriltag Cam",
                0.6,
                5,
                new Transform3d(
                    new Translation3d(
                        Inches.of(+12.225),
                        Inches.of(+9.557),
                        Inches.of(+10.932)
                    ),
                    new Rotation3d(
                        Units.degreesToRadians(0),
                        Units.degreesToRadians(-12.348-5),
                        Units.degreesToRadians(+0)
                    )
                ),
                (connected) -> {
                    Leds.getInstance().lAprilConnected = connected;
                }
                // robotToCameraFromCalibTag(
                //     new Transform3d(
                //         new Translation3d(
                //             4.62,
                //             0,
                //             1.465
                //         ),
                //         new Rotation3d(
                //             0,0,Math.PI
                //         )
                //     ),
                //     new Transform3d(
                //         new Translation3d(
                //             4.26,
                //             -0.126,
                //             1.145
                //         ),
                //         new Rotation3d(
                //             new Quaternion(
                //                 0.033,
                //                 0.03,
                //                 0,
                //                 1
                //             )
                //         )
                //     )
                // )
            ),
            RightApriltag(
                "Right Apriltag Cam",
                1,
                4,
                new Transform3d(
                    new Translation3d(
                        Inches.of(+12.225),
                        Inches.of(-9.557),
                        Inches.of(+10.932)
                    ),
                    new Rotation3d(
                        Units.degreesToRadians(0),
                        Units.degreesToRadians(-32.414),
                        Units.degreesToRadians(0)
                    )
                    // new Rotation3d(
                    //     Units.degreesToRadians(-(90.0-87.654)),
                    //     Units.degreesToRadians(-32.414),
                    //     Units.degreesToRadians(-9.707)
                    // )
                ),
                (connected) -> {
                    Leds.getInstance().rAprilConnected = connected;
                }
            ),
            NoteVision(
                "Note Cam",
                0,
                0,
                new Transform3d(
                    new Translation3d(
                        Inches.of(-14.047),
                        Inches.of(+0),
                        Inches.of(+12.584)
                    ),
                    new Rotation3d(
                        0,
                        Degrees.of(15).in(Radians),
                        Math.PI
                    )
                ),
                (connected) -> {
                    Leds.getInstance().nVisionConnected = connected;
                }
            ),
            ;
            public final String hardwareName;
            private final Transform3d intermediateToCamera;
            public final double cameraStdCoef;
            public final double trustDistance;
            public final BooleanConsumer connectedConsumer;
            private Supplier<Transform3d> robotToIntermediate;
            Camera(String hardwareName, double cameraStdCoef, double trustDistance, Transform3d finalToCamera, BooleanConsumer connectedConsumer) {
                this.hardwareName = hardwareName;
                this.cameraStdCoef = cameraStdCoef;
                this.trustDistance = trustDistance;
                this.intermediateToCamera = finalToCamera;
                this.connectedConsumer = connectedConsumer;
                this.robotToIntermediate = Transform3d::new;
            }
            @SuppressWarnings("unused")
            private static Transform3d robotToCameraFromCalibTag(Transform3d robotToCalibTag, Transform3d cameraToCalibTag) {
                return robotToCalibTag.plus(cameraToCalibTag.inverse());
            }
            public Camera withRobotToIntermediate(Supplier<Transform3d> robotToFinal) {
                this.robotToIntermediate = robotToFinal;
                return this;
            }

            public Transform3d getRobotToCam() {
                return robotToIntermediate.get().plus(intermediateToCamera);
            }

            public ApriltagCamera toApriltagCamera() {
                return new ApriltagCamera(this, new ApriltagCameraIO(){});
            }
            public ApriltagCamera toApriltagCamera(Function<Camera, ? extends ApriltagCameraIO> function) {
                return new ApriltagCamera(this, function.apply(this));
            }

            public static void logCameraOverrides() {
                Logger.recordOutput("Camera Overrides", 
                    Arrays.stream(Camera.values())
                    .map(
                        (cam) -> 
                            new Transform3d(
                                new Pose3d(),
                                new Pose3d(RobotState.getInstance().getPose())
                            )
                            .plus(cam.getRobotToCam())
                    )
                    .toArray(Transform3d[]::new)
                );
            }
        }

        // TODO: figure out vision stdDevs
        public static final double singleTagAmbiguityCutoff = 0.05;
        public static final double minimumStdDev = 0.5;
        public static final double stdDevEulerMultiplier = 0.3;
        public static final double stdDevDistanceMultiplier = 0.4;
    }

    public static final class AutoConstants {
        public static final double allottedAutoTime = 15.3;

        public static final double maxVel = 3;
        public static final double maxAccel = 3;

        public static final double maxVelFast = 4;
        public static final double maxAccelFast = 4.5;

        public static final double maxVelSlow = 0.75;
        public static final double maxAccelSlow = 1.5;

        public static final double autoTranslationXKp = 11;
        public static final double autoTranslationXKi = 0;
        public static final double autoTranslationXKd = 0;

        public static final double autoTranslationYKp = 8;
        public static final double autoTranslationYKi = 0;
        public static final double autoTranslationYKd = 0;

        public static final double autoTranslationSlowXKp = 8;
        public static final double autoTranslationSlowXKi = 0;
        public static final double autoTranslationSlowXKd = 0;

        public static final double autoTranslationSlowYKp = 6;
        public static final double autoTranslationSlowYKi = 0;
        public static final double autoTranslationSlowYKd = 0;

        public static final double autoRotationKp = 8;
        public static final double autoRotationKi = 0;
        public static final double autoRotationKd = 0;
    }

    public static final class FieldConstants {
        public static final Distance fieldLength = Inches.of(648);
        public static final Distance fieldWidth =  Inches.of(324);

        public static final FlippedTranslation3d speakerAimPoint = FlippedTranslation3d.fromBlue(new Translation3d(0.240581, 5.547755, 2));
        public static final FlippedTranslation3d passAimPoint =    FlippedTranslation3d.fromBlue(speakerAimPoint.getBlue().interpolate(new Translation3d(1.83,7.61,2), 0.7));

        public static final FlippedPose2d subwooferFront =  FlippedPose2d.fromBlue(new Pose2d(new Translation2d(1.45, 5.55), Rotation2d.fromDegrees(+180)));
        public static final FlippedPose2d subwooferAmp =    FlippedPose2d.fromBlue(new Pose2d(new Translation2d(0.71, 6.72), Rotation2d.fromDegrees(-120)));
        public static final FlippedPose2d subwooferSource = FlippedPose2d.fromBlue(new Pose2d(new Translation2d(0.71, 6.72), Rotation2d.fromDegrees(+120)));
        
        public static final FlippedPose2d amp =    FlippedPose2d.fromBlue(new Pose2d(new Translation2d(1.83, 7.61), Rotation2d.fromDegrees(-90)));
        public static final FlippedPose2d podium = FlippedPose2d.fromBlue(new Pose2d(new Translation2d(2.76, 4.44), Rotation2d.fromDegrees(+157.47)));

        public static final FlippedPose2d pathfindSpeaker = FlippedPose2d.fromBlue(new Pose2d(new Translation2d(3.45, 5.55), Rotation2d.fromDegrees(+180)));
        public static final FlippedPose2d pathfindSource =  FlippedPose2d.fromBlue(new Pose2d(new Translation2d(13.41, 1.54), Rotation2d.fromDegrees(+180)));

        public static final double podiumToSpeakerDist =    speakerAimPoint.getBlue().toTranslation2d().getDistance(podium.getBlue().getTranslation());
        public static final double subwooferToSpeakerDist = speakerAimPoint.getBlue().toTranslation2d().getDistance(subwooferFront.getBlue().getTranslation());
    }
}
