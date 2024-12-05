package frc.robot.subsystems.vision.apriltag;

import java.util.Arrays;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.vision.apriltag.ApriltagVisionConstants.ApriltagCameraConstants;
import frc.robot.subsystems.vision.apriltag.LimelightHelpers.LimelightTarget_Fiducial;
import frc.util.GeomUtil;

@Deprecated
public class ApriltagCameraIOLimelight implements ApriltagCameraIO {

    private final String cameraName; 

    public ApriltagCameraIOLimelight(ApriltagCameraConstants cameraData) {
        // Important: need to configure robotToCamera pose using Limelight webUI
        // Important: need to configure AprilTag field map using Limelight webUI
        // https://docs.limelightvision.io/en/latest/apriltags_in_3d.html#robot-localization-botpose-and-megatag
        this.cameraName = cameraData.hardwareName;
        LimelightHelpers.setPipelineIndex(cameraName, 0);
    }

    @Override
    public void updateInputs(ApriltagCameraIOInputs inputs) {
        inputs.isConnected = false;

        // get parsed results from JSON on NetworkTables.  
        // Use this JSON results to make sure all values are from the same snapshot
        LimelightHelpers.Results result = LimelightHelpers.getLatestResults(cameraName).targetingResults;

        // TODO: figure out how to determine if Limelight is disconnected
        inputs.isConnected = true;

        if (!inputs.isConnected || LimelightHelpers.getFiducialID(cameraName) < 0) return;

        double latencySeconds = (result.latency_capture + result.latency_pipeline + result.latency_jsonParse) / 1000.0;
        var timestamp = Timer.getFPGATimestamp() - latencySeconds;
        inputs.timestamp = timestamp;

        inputs.targets = Arrays.stream(result.targets_Fiducials).map(ApriltagCameraIOLimelight::targetFromLLTarget).toArray(ApriltagCameraTarget[]::new);
        inputs.estimatedRobotPose = result.getBotPose3d_wpiBlue();
    }

    private static ApriltagCameraTarget targetFromLLTarget(LimelightTarget_Fiducial limelightTarget) {
        return new ApriltagCameraTarget(
            (int) limelightTarget.fiducialID,
            GeomUtil.toTransform3d(limelightTarget.getTargetPose_CameraSpace()),
            GeomUtil.toTransform3d(limelightTarget.getTargetPose_CameraSpace()),
            0
        );
    }
}
