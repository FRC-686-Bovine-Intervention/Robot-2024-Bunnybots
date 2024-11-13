package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import frc.util.loggerUtil.LoggedEncoder;
import frc.util.loggerUtil.LoggedMotor;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs {
        LoggedMotor motor = new LoggedMotor();
        LoggedEncoder encoder = new LoggedEncoder();
    }

    public default void updateInputs(ArmIOInputs inputs) {}

    public default void setVoltage(Measure<VoltageUnit> volts) {}

    public default void setPos(Measure<AngleUnit> pos) {}

    public default void setOffset(Measure<AngleUnit> offset) {}

    public default void setCoast(boolean coast) {}

    public default void stop() {}

    public default void zero() {}
}
