package frc.robot.constants;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;
import frc.robot.Robot;

public final class RobotConstants {
    public static final boolean tuningMode = true;

    /**Distance between back bumper and front bumper, aka in the X axis */
    public static final Distance robotLength = Centimeters.of(90);
    /**Distance between left bumper and right bumper, aka in the Y axis */
    public static final Distance robotWidth = Centimeters.of(90);

    public static final Distance centerToBumperCorner = Meters.of(new Translation2d(robotLength, robotWidth).getNorm());

    public static final double rioUpdatePeriodSecs = Robot.defaultPeriodSecs;
    public static final Time rioUpdatePeriod = Seconds.of(rioUpdatePeriodSecs);
    public static final Frequency rioUpdateFrequency = rioUpdatePeriod.asFrequency();
    public static final double rioUpdateFrequencyHz = rioUpdateFrequency.in(Hertz);
}
