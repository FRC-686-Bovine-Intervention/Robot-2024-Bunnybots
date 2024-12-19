package frc.robot.subsystems.vision.apriltag;

import org.photonvision.simulation.PhotonCameraSim;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.apriltag.ApriltagVisionConstants.ApriltagCameraConstants;

public class ApriltagCameraSim extends ApriltagCamera {
    public ApriltagCameraSim(ApriltagCameraConstants camMeta, ApriltagCameraIOSim io) {
        super(camMeta, io);
    }

    public Transform3d getRobotRelative() {
        return camMeta.mount.getRobotRelative();
    }

    public PhotonCameraSim getCameraSim() {
        return ((ApriltagCameraIOSim) io).camSim;
    }
}
