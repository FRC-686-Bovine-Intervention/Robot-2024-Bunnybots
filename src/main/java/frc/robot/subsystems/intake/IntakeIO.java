package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Millimeters;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.MutDistance;
import frc.util.loggerUtil.LoggedMotor;

public interface IntakeIO {
    
    @AutoLog
    public static class IntakeIOInputs {
        public LoggedMotor motor = new LoggedMotor();
        public MutDistance proximity = Millimeters.mutable(5000);
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setMotorVoltage(Measure<VoltageUnit> volts) {}

    public default void setOpen(boolean open) {}
}
