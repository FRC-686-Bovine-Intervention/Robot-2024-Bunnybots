package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.loggerUtil.LoggedEncodedMotor;

public interface ModuleIO {

    @AutoLog
    public static class ModuleIOInputs {
        public LoggedEncodedMotor driveMotor = new LoggedEncodedMotor();
        public LoggedEncodedMotor turnMotor = new LoggedEncodedMotor();
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ModuleIOInputs inputs) {}

    /** Run the drive motor at the specified voltage. */
    public default void setDriveVoltage(Voltage volts) {}

    /** Run the turn motor at the specified voltage. */
    public default void setTurnVoltage(Voltage volts) {}

    /** Enable or disable brake mode on the drive motor. */
    public default void setDriveBrakeMode(Boolean enable) {}

    /** Enable or disable brake mode on the turn motor. */
    public default void setTurnBrakeMode(Boolean enable) {}

    public default void stop() {}

    /** Zero drive encoders */
    public default void zeroEncoders() {}
}
