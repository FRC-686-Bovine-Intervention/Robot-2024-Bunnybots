package frc.robot.subsystems.vision.apriltag;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants.VisionConstants.Camera;
import frc.robot.RobotState;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

public class ApriltagCameraIOPhotonVision implements ApriltagCameraIO {

    private final PhotonCamera photonCam;
    private final Camera cam;
    private PhotonPoseEstimator photonPoseEstimator;

    public ApriltagCameraIOPhotonVision(Camera cam) {
        this.cam = cam;
        photonCam = new PhotonCamera(cam.hardwareName);

        var fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

        photonPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCam, cam.getRobotToCam());
        photonPoseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE);

        notConnectedAlert = new Alert(cam.hardwareName + " is not connected", AlertType.ERROR);
    }

    private final Alert notConnectedAlert;
    public void updateInputs(ApriltagCameraIOInputs inputs) {
        // set default values
        inputs.isConnected = photonCam.isConnected();
        notConnectedAlert.set(!inputs.isConnected);
        inputs.hasResult = false;

        if ((!inputs.isConnected) || (photonPoseEstimator == null)) return;

        photonPoseEstimator.setRobotToCameraTransform(cam.getRobotToCam());
        photonPoseEstimator.setReferencePose(RobotState.getInstance().getPose());

        var result = photonCam.getLatestResult();
        var optRobotPose = photonPoseEstimator.update(result);
        
        optRobotPose.ifPresent((e) -> {
            Logger.recordOutput("DEBUG/VISION/" + cam.name(), result.getBestTarget().getBestCameraToTarget());
            inputs.hasResult = true;
            inputs.timestamp = e.timestampSeconds;
            inputs.estimatedRobotPose = e.estimatedPose;
            inputs.cameraToTagDist = e.targetsUsed.stream().map(PhotonTrackedTarget::getBestCameraToTarget).map(Transform3d::getTranslation).mapToDouble(Translation3d::getNorm).toArray();
            inputs.tagsSeen = e.targetsUsed.stream().mapToInt(PhotonTrackedTarget::getFiducialId).toArray();
        });
    }
}
