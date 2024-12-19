package frc.robot.subsystems.vision.bucket;

import org.photonvision.simulation.SimCameraProperties;

import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionConstants.CameraConstants;
import frc.util.robotStructure.CameraMount;

public class BucketVisionConstants {
    public static class BucketCameraConstants extends CameraConstants {

        public BucketCameraConstants(String hardwareName, CameraMount mount, SimCameraProperties simCamProps) {
            super(hardwareName, mount, simCamProps);
        }
    }

    public static final BucketCameraConstants bucketCamera = new BucketCameraConstants(
        "Bucket Cam",
        VisionConstants.flagStickMount,
        VisionConstants.bucketSimCamProps
    );
}
