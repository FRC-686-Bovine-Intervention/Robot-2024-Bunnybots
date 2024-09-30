package frc.robot.subsystems.vision.apriltag;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.VisionConstants.Camera;

public class ApriltagCameraIOLimelight implements ApriltagCameraIO {

    private final String cameraName; 

    public ApriltagCameraIOLimelight(Camera cameraData) {
        // Important: need to configure robotToCamera pose using Limelight webUI
        // Important: need to configure AprilTag field map using Limelight webUI
        // https://docs.limelightvision.io/en/latest/apriltags_in_3d.html#robot-localization-botpose-and-megatag
        this.cameraName = cameraData.hardwareName;
        LimelightHelpers.setPipelineIndex(cameraName, 0);
    }

    public void updateInputs(ApriltagCameraIOInputs inputs) {
        // set default values
        inputs.isConnected = false;
        inputs.hasResult = false;

        // get parsed results from JSON on NetworkTables.  
        // Use this JSON results to make sure all values are from the same snapshot
        LimelightHelpers.Results result = LimelightHelpers.getLatestResults(cameraName).targetingResults;

        // TODO: figure out how to determine if Limelight is disconnected
        inputs.isConnected = true;

        if (!inputs.isConnected || LimelightHelpers.getFiducialID(cameraName) < 0) return;

        var cameraToTargetDist = LimelightHelpers.getTargetPose3d_CameraSpace(cameraName).getTranslation().getNorm();
        var visionPose = result.getBotPose3d_wpiBlue();
        double latencySeconds = (result.latency_capture + result.latency_pipeline + result.latency_jsonParse) / 1000.0;
        var timestamp = Timer.getFPGATimestamp() - latencySeconds;

        inputs.hasResult = true;
        // inputs.result = new ApriltagCameraResult(timestamp, cameraToTargetDist, visionPose);
    }
}
