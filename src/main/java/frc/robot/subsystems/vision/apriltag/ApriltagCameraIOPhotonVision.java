package frc.robot.subsystems.vision.apriltag;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.RobotState;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.vision.VisionConstants.CameraConstants;

public class ApriltagCameraIOPhotonVision implements ApriltagCameraIO {
    private final PhotonCamera photonCam;
    private final CameraConstants camMeta;
    private PhotonPoseEstimator photonPoseEstimator;

    public ApriltagCameraIOPhotonVision(CameraConstants cam) {
        this.camMeta = cam;
        photonCam = new PhotonCamera(cam.hardwareName);

        photonPoseEstimator = new PhotonPoseEstimator(FieldConstants.apriltagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCam, Transform3d.kZero);
        photonPoseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
    }

    public void updateInputs(ApriltagCameraIOInputs inputs) {
        inputs.isConnected = photonCam.isConnected();

        if (!inputs.isConnected) return;
        var result = photonCam.getLatestResult();
        inputs.targets = result.getTargets().stream().map(ApriltagCameraTarget::fromPhotonTarget).toArray(ApriltagCameraTarget[]::new);

        if (photonPoseEstimator == null) return;
        photonPoseEstimator.setRobotToCameraTransform(camMeta.mount.getRobotRelative());
        photonPoseEstimator.setReferencePose(RobotState.getInstance().getPose());

        var optRobotPose = photonPoseEstimator.update(result);
        optRobotPose.ifPresent((e) -> {
            inputs.timestamp = e.timestampSeconds;
            inputs.estimatedRobotPose = e.estimatedPose;
        });
    }
}
