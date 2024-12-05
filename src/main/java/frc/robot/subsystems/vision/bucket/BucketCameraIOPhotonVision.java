package frc.robot.subsystems.vision.bucket;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
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
        Logger.recordOutput("DEBUG/photon pitch raw", photonTarget.getPitch());
        return new BucketCameraTarget(
            new Translation3d(
                1,
                0,
                0
            ).rotateBy(new Rotation3d(
                0,
                0,
                Radians.convertFrom(-photonTarget.getYaw(), Degrees)
            )).rotateBy(new Rotation3d(
                0,
                Radians.convertFrom(-photonTarget.getPitch(), Degrees),
                0
            )),
            photonTarget.getArea()
        );
    }
}
