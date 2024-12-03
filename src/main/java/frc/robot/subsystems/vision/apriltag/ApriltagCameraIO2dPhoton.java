package frc.robot.subsystems.vision.apriltag;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.vision.VisionConstants.ApriltagCameraConstants;

public class ApriltagCameraIO2dPhoton implements ApriltagCameraIO {
    private final PhotonCamera photonCam;
    private final ApriltagCameraConstants camMeta;

    public ApriltagCameraIO2dPhoton(ApriltagCameraConstants camMeta) {
        this.camMeta = camMeta;
        this.photonCam = new PhotonCamera(camMeta.hardwareName);
    }

    public void updateInputs(ApriltagCameraIOInputs inputs) {
        // set default values
        inputs.isConnected = photonCam.isConnected();
        inputs.hasResult = false;

        if (!inputs.isConnected) {
            return;
        }
        var result = photonCam.getLatestResult();
        Logger.recordOutput("DEBUG/Tag Field Poses", FieldConstants.apriltagLayout.getTags().stream().map((tag) -> tag.pose).toArray(Pose3d[]::new));
        Logger.recordOutput("DEBUG/photonresult", result);
        Logger.recordOutput("DEBUG/tagposes", result.getTargets().stream().map(this::resultToTargets).toArray(Pose3d[]::new));
    }

    private static final double yawCalib = 0;//-21.88;
    private static final double pitchCalib = 0;//-19.41;

    private Pose3d resultToTargets(PhotonTrackedTarget target) {
        var tagPos = FieldConstants.apriltagLayout.getTagPose(target.getFiducialId()).get();
        var pitch = -target.getYaw();
        var yaw = target.getPitch();
        var targetCamViewTransform = camMeta.mount.getRobotRelative().plus(
            new Transform3d(
                new Translation3d(),
                new Rotation3d(
                    0,
                    Units.degreesToRadians(pitch - pitchCalib),
                    Units.degreesToRadians(yaw - yawCalib)
                )
            )
        );
        var distOut = (tagPos.getZ() - targetCamViewTransform.getTranslation().getZ()) / Math.tan(targetCamViewTransform.getRotation().getY());
        var distOff = distOut * Math.tan(targetCamViewTransform.getRotation().getZ());
        var camToTargetTranslation = new Translation3d(distOut, distOff, tagPos.getZ()-camMeta.mount.getRobotRelative().getZ());
        var fieldPos = tagPos.transformBy(new Transform3d(camToTargetTranslation, new Rotation3d()).inverse()).transformBy(new Transform3d(camMeta.mount.getRobotRelative().getTranslation(), new Rotation3d()).inverse());
        // var fieldPos = new Pose3d(RobotState.getInstance().getPose())
        //     .transformBy(new Transform3d(camToTargetTranslation, new Rotation3d()))
        //     .transformBy(camMeta.getRobotToCam())
        //     ;
        // var fieldPos = new Pose3d().transformBy(new Transform3d(camToTargetTranslation, new Rotation3d()));
        return fieldPos;
    }
}
