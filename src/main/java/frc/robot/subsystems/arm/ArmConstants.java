package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.util.GearRatio;

public class ArmConstants {
    public static final Frequency IOSignalFrequency = Hertz.of(50);

    public static final GearRatio motorToMechanismRatio = new GearRatio();
    public static final GearRatio encoderToMechanismRatio = new GearRatio();
    public static final GearRatio motorToEncoderRatio = motorToMechanismRatio.concat(encoderToMechanismRatio.inverse());

    public static final Distance length = Inches.of(0);
    public static final Angle startAngle = Degrees.of(0);
    public static final Angle minAngle = Degrees.of(-10);
    public static final Angle maxAngle = Degrees.of(120);
    public static final MomentOfInertia momentOfInertia = KilogramSquareMeters.of(0.1);
}
