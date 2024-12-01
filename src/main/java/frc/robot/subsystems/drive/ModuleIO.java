package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import frc.util.loggerUtil.LoggedEncodedMotor;

public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public LoggedEncodedMotor driveMotor = new LoggedEncodedMotor();
        public LoggedEncodedMotor turnMotor = new LoggedEncodedMotor();
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ModuleIOInputs inputs) {}

    /** Run the drive motor at the specified voltage. */
    public default void setDriveVoltage(Measure<VoltageUnit> volts) {}
    public default void setDriveVelocity(Measure<AngularVelocityUnit> velocity) {}

    /** Run the turn motor at the specified voltage. */
    public default void setTurnVoltage(Measure<VoltageUnit> volts) {}
    public default void setTurnAngle(Measure<AngleUnit> angle) {}

    /** Enable or disable brake mode on the drive motor. */
    public default void setDriveBrakeMode(boolean enable) {}

    /** Enable or disable brake mode on the turn motor. */
    public default void setTurnBrakeMode(boolean enable) {}

    public default void stop() {}

    /** Zero drive encoders */
    // public default void zeroEncoders() {}
}
