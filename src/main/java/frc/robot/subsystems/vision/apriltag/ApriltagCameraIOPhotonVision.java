package frc.robot.subsystems.vision.apriltag;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.RobotState;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.vision.VisionConstants.ApriltagCameraConstants;

public class ApriltagCameraIOPhotonVision implements ApriltagCameraIO {

    private final PhotonCamera photonCam;
    private final ApriltagCameraConstants cam;
    private PhotonPoseEstimator photonPoseEstimator;

    public ApriltagCameraIOPhotonVision(ApriltagCameraConstants cam) {
        this.cam = cam;
        photonCam = new PhotonCamera(cam.hardwareName);

        photonPoseEstimator = new PhotonPoseEstimator(FieldConstants.apriltagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCam, Transform3d.kZero);
        photonPoseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE);

        notConnectedAlert = new Alert(cam.hardwareName + " is not connected", AlertType.kError);
    }

    private final Alert notConnectedAlert;
    public void updateInputs(ApriltagCameraIOInputs inputs) {
        // set default values
        inputs.isConnected = photonCam.isConnected();
        notConnectedAlert.set(!inputs.isConnected);
        // cam.connectedConsumer.accept(inputs.isConnected);
        inputs.hasResult = false;

        if ((!inputs.isConnected) || (photonPoseEstimator == null)) return;

        photonPoseEstimator.setRobotToCameraTransform(cam.mount.getRobotRelative());
        photonPoseEstimator.setReferencePose(RobotState.getInstance().getPose());

        var result = photonCam.getLatestResult();
        var optRobotPose = photonPoseEstimator.update(result);
        
        optRobotPose.ifPresent((e) -> {
            Logger.recordOutput("DEBUG/VISION/" + cam.hardwareName, result.getBestTarget().getBestCameraToTarget());
            inputs.hasResult = true;
            inputs.timestamp = e.timestampSeconds;
            inputs.estimatedRobotPose = e.estimatedPose;
            inputs.cameraToTagDist = e.targetsUsed.stream().map(PhotonTrackedTarget::getBestCameraToTarget).map(Transform3d::getTranslation).mapToDouble(Translation3d::getNorm).toArray();
            inputs.tagsSeen = e.targetsUsed.stream().mapToInt(PhotonTrackedTarget::getFiducialId).toArray();
        });
    }
}
