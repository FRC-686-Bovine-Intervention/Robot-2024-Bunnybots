package frc.robot.subsystems.vision.apriltag;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.subsystems.vision.VisionConstants.CameraConstants;

public class ApriltagCameraIOSim implements ApriltagCameraIO {
    private final PhotonCamera photonCam;
    public final PhotonCameraSim camSim;

    public ApriltagCameraIOSim(CameraConstants cam) {
        this.photonCam = new PhotonCamera(cam.hardwareName);
        this.camSim = new PhotonCameraSim(photonCam, cam.simCamProps);
    }
    
    @Override
    public void updateInputs(ApriltagCameraIOInputs inputs) {
        inputs.isConnected = photonCam.isConnected();

        if (!inputs.isConnected) return;
        var result = photonCam.getLatestResult();
        inputs.targets = result
            .getTargets()
            .stream()
            .map(ApriltagCameraIOSim::targetFromPhotonTarget)
            .toArray(ApriltagCameraTarget[]::new);
    }

    private static ApriltagCameraTarget targetFromPhotonTarget(PhotonTrackedTarget photonTarget) {
        return new ApriltagCameraTarget(
            photonTarget.getFiducialId(),
            photonTarget.getBestCameraToTarget(),
            photonTarget.getAlternateCameraToTarget(),
            photonTarget.getPoseAmbiguity()
        );
    }
}
