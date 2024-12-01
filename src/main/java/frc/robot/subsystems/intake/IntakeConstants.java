package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public final class IntakeConstants {
    

    public static final boolean pistonInverted = false;

    public static final Distance sensorThreshold = Inches.of(5);

    public static final Angle clawMotion = Radians.of(0.183023);
}
