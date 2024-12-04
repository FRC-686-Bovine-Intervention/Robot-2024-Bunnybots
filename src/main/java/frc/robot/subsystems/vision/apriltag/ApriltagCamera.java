package frc.robot.subsystems.vision.apriltag;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.subsystems.vision.apriltag.ApriltagCameraIO.ApriltagCameraIOInputs;
import frc.robot.subsystems.vision.apriltag.ApriltagCameraIO.ApriltagCameraTarget;
import frc.robot.subsystems.vision.apriltag.ApriltagVisionConstants.ApriltagCameraConstants;

public class ApriltagCamera {
    private final ApriltagCameraConstants camMeta;
    private final ApriltagCameraIO io;
    private final ApriltagCameraIOInputsAutoLogged inputs = new ApriltagCameraIOInputsAutoLogged();

    private final Alert notConnectedAlert;

    public ApriltagCamera(ApriltagCameraConstants camMeta, ApriltagCameraIO io) {
        this.camMeta = camMeta;
        this.io = io;

        notConnectedAlert = new Alert("Apriltag camera \"" + camMeta.hardwareName + "\" is not connected", AlertType.kError);
    }

    public Optional<ApriltagCameraResult> periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/ApriltagVision/" + camMeta.hardwareName, inputs);

        notConnectedAlert.set(!inputs.isConnected);
        return ApriltagCameraResult.from(camMeta, inputs);
    }

    public static class ApriltagCameraResult {
        public final ApriltagCameraConstants camMeta;
        public final double timestamp;
        public final ApriltagCameraTarget[] targets;
        public final Pose3d estimatedRobotPose;

        private ApriltagCameraResult(ApriltagCameraConstants camMeta, double timestamp, ApriltagCameraTarget[] targets, Pose3d estimatedRobotPose) {
            this.camMeta = camMeta;
            this.timestamp = timestamp;
            this.targets = targets;
            this.estimatedRobotPose = estimatedRobotPose;
        }

        public static Optional<ApriltagCameraResult> from(ApriltagCameraConstants camMeta, ApriltagCameraIOInputs inputs) {
            if (!inputs.isConnected || inputs.targets.length <= 0) return Optional.empty();

            return Optional.of(new ApriltagCameraResult(
                camMeta,
                inputs.timestamp,
                inputs.targets,
                inputs.estimatedRobotPose
            ));
        }
    }
}