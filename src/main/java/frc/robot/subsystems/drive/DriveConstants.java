package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import java.util.Arrays;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.constants.CANDevices;
import frc.util.Environment;
import frc.util.GearRatio;
import frc.util.LoggedTunableNumber;
import frc.util.MathExtraUtil;
import frc.util.GearRatio.Wheel;

public final class DriveConstants {
    /**Distance between the front and back wheels*/
    public static final Distance trackWidthX = Inches.of(25.5);
    /**Distance between the left and right wheels*/
    public static final Distance trackWidthY = Inches.of(25.5);

    public static class ModuleConstants {
        public final String name;
        public final int driveMotorID;
        public final int turnMotorID;
        // motor direction to drive 'forward' (cancoders at angles given in cancoderOffsetRotations)
        public final InvertedValue driveInverted;
        // absolute position of cancoder when drive wheel is facing 'forward'
        public final Angle cancoderOffset;
        public final Translation2d moduleTranslation;
        public final Vector<N2> positiveRotVec;
        ModuleConstants(String name, int driveMotorID, int turnMotorID, InvertedValue driveInverted, Angle cancoderOffset, Translation2d moduleTranslation) {
            this.name = name;
            this.driveMotorID = driveMotorID;
            this.turnMotorID = turnMotorID;
            this.driveInverted = driveInverted;
            this.cancoderOffset = cancoderOffset;
            this.moduleTranslation = moduleTranslation;
            this.positiveRotVec = MathExtraUtil.vectorFromRotation(this.moduleTranslation.getAngle().plus(Rotation2d.fromDegrees(90)));
        }
    }

    public static final ModuleConstants[] modules = {
        new ModuleConstants(
            "Front Left",
            CANDevices.frontLeftDriveMotorID, CANDevices.frontLeftTurnMotorID,
            InvertedValue.CounterClockwise_Positive,
            Rotations.of(0.75),
            new Translation2d(
                trackWidthX.divide(+2),
                trackWidthY.divide(+2)
            )
        ),
        new ModuleConstants(
            "Front Right",
            CANDevices.frontRightDriveMotorID, CANDevices.frontRightTurnMotorID,
            InvertedValue.Clockwise_Positive,
            Rotations.of(0.5),
            new Translation2d(
                trackWidthX.divide(+2),
                trackWidthY.divide(-2)
            )
        ),
        new ModuleConstants(
            "Back Left",
            CANDevices.backLeftDriveMotorID, CANDevices.backLeftTurnMotorID,
            InvertedValue.CounterClockwise_Positive,
            Rotations.of(0.5),
            new Translation2d(
                trackWidthX.divide(-2),
                trackWidthY.divide(+2)
            )
        ),
        new ModuleConstants(
            "Back Right",
            CANDevices.backRightDriveMotorID, CANDevices.backRightTurnMotorID,
            InvertedValue.Clockwise_Positive,
            Rotations.of(0.75),
            new Translation2d(
                trackWidthX.divide(-2),
                trackWidthY.divide(-2)
            )
        ),
    };
    public static final Translation2d[] moduleTranslations = Arrays.stream(modules).map((a) -> a.moduleTranslation).toArray(Translation2d[]::new);

    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);

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
    public static final AngularVelocity maxTurnRate = RadiansPerSecond.of(maxDriveSpeed.in(MetersPerSecond) / new Translation2d(trackWidthX.divide(2), trackWidthY.divide(2)).getNorm());
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
    public static final Angle headingTolerance = Degrees.of(1);
    public static final AngularVelocity omegaTolerance = DegreesPerSecond.of(1);
}