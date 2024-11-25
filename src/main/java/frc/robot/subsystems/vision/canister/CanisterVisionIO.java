package frc.robot.subsystems.vision.canister;

import org.littletonrobotics.junction.AutoLog;

public interface CanisterVisionIO {
    @AutoLog
    public static class CanisterVisionIOInputs {
        public boolean connected;
    }

    public default void updateInputs(CanisterVisionIOInputs inputs) {}
}
