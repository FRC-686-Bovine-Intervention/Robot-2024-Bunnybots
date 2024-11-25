package frc.util.loggerUtil;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.subsystems.vision.VisionConstants.Camera;

public class LoggedGroundVisionTarget implements StructSerializable {
    public Translation2d fieldPos;
    public double confidence;

    LoggedGroundVisionTarget(Camera camera) {}

    public void updateFrom(Camera camera, PhotonTrackedTarget target) {
        var robotToCam = camera.getRobotToCam();
        var pitch = target.getPitch();
        var yaw = target.getYaw();

        var distOut = robotToCam.getZ() / Math.tan(pitch);
        var distOff = distOut * Math.tan(yaw);
        var camToTargetTranslation = new Translation3d(distOut, distOff, -robotToCam.getZ());
    }
}
