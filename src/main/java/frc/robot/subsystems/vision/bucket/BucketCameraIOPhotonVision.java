package frc.robot.subsystems.vision.bucket;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.subsystems.vision.VisionConstants.CameraConstants;

public class BucketCameraIOPhotonVision implements BucketCameraIO {
    private final PhotonCamera photonCam;
    private final CameraConstants camMeta;

    public BucketCameraIOPhotonVision(CameraConstants cam) {
        this.camMeta = cam;
        photonCam = new PhotonCamera(this.camMeta.hardwareName);
    }

    @Override
    public void updateInputs(BucketCameraIOInputs inputs) {
        inputs.isConnected = photonCam.isConnected();

        if (!inputs.isConnected) return;
        var result = photonCam.getLatestResult();
        inputs.targets = result.getTargets().stream().map(BucketCameraIOPhotonVision::targetFromPhotonTarget).toArray(BucketCameraTarget[]::new);
    }

    private static BucketCameraTarget targetFromPhotonTarget(PhotonTrackedTarget photonTarget) {
        return new BucketCameraTarget(
            new Rotation3d(
                0,
                photonTarget.getPitch(),
                photonTarget.getPitch()
            ),
            photonTarget.getArea()
        );
    }
}
