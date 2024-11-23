package frc.robot.subsystems.puncher;

import org.littletonrobotics.junction.AutoLog;

public interface PuncherIO {
    @AutoLog
    public static class PuncherIOInputs {}

    public default void updateInputs(PuncherIOInputs inputs) {}

    public default void extend() {}
    
    public default void retract() {}

    public default void toggle() {}
}
