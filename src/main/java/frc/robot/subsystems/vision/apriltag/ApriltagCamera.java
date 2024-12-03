package frc.robot.subsystems.vision.apriltag;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.vision.VisionConstants.ApriltagCameraConstants;
import frc.robot.subsystems.vision.apriltag.ApriltagCameraIO.ApriltagCameraResult;

public class ApriltagCamera {
    private final ApriltagCameraConstants cameraMeta;
    private final ApriltagCameraIO cameraIO;
    private final ApriltagCameraIOInputsAutoLogged inputs = new ApriltagCameraIOInputsAutoLogged();

    public ApriltagCamera(ApriltagCameraConstants cameraMeta, ApriltagCameraIO cameraIO) {
        this.cameraMeta = cameraMeta;
        this.cameraIO = cameraIO;
    }

    public Optional<ApriltagCameraResult> periodic() {
        cameraIO.updateInputs(inputs);
        Logger.processInputs("Inputs/ApriltagVision/" + cameraMeta.hardwareName, inputs);
        return ApriltagCameraResult.from(cameraMeta, inputs);
    }
}