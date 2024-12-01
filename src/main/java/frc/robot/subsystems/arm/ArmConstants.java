package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.util.GearRatio;
import frc.util.LoggedTunableMeasure;

public final class ArmConstants {
    public static final Frequency MotorSignalFrequency = Hertz.of(50);

    public static final GearRatio motorToMechanismRatio = new GearRatio()
        .planetary(1.0/5)
        .planetary(1.0/5)
        // .planetary(1/4)
        .sprocket(16).sprocket(64)
    ;
    public static final GearRatio encoderToMechanismRatio = new GearRatio();
    public static final GearRatio motorToEncoderRatio = motorToMechanismRatio.concat(encoderToMechanismRatio.inverse());

    public static final Distance length = Inches.of(15);
    public static final Angle startAngle = Degrees.of(0);
    public static final Angle minAngle = Degrees.of(0);
    public static final Angle maxAngle = Degrees.of(105);
    public static final MomentOfInertia momentOfInertia = KilogramSquareMeters.of(0.005);

    public static final LoggedTunableMeasure<AngleUnit> tolerance = new LoggedTunableMeasure<>("Arm/Tolerance", Degrees.of(2));
}
