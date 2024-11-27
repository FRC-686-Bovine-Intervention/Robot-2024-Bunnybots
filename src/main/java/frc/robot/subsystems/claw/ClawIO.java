package frc.robot.subsystems.claw;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import frc.robot.util.loggerUtil.LoggedMotor;

public interface ClawIO {
    @AutoLog
    public static class ClawIOInputs {
        LoggedMotor motor = new LoggedMotor();
    }

    public default void updateInputs(ClawIOInputs inputs) {}

    public default void setMotorVoltage(Measure<VoltageUnit> volts) {}

    public default void open() {}

    public default void close() {}
}
