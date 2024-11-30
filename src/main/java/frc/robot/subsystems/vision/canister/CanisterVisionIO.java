package frc.robot.subsystems.vision.canister;

import org.littletonrobotics.junction.AutoLog;

import frc.util.loggerUtil.LoggedGroundVisionTarget;

public interface CanisterVisionIO {
    @AutoLog
    public static class CanisterVisionIOInputs {
        public LoggedGroundVisionTarget[] trackedCanisters = new LoggedGroundVisionTarget[0];
        public boolean connected;
    }

    public default void updateInputs(CanisterVisionIOInputs inputs) {}
}
