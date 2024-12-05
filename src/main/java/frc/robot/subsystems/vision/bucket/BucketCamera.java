package frc.robot.subsystems.vision.bucket;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.subsystems.vision.bucket.BucketCameraIO.BucketCameraIOInputs;
import frc.robot.subsystems.vision.bucket.BucketCameraIO.BucketCameraTarget;
import frc.robot.subsystems.vision.bucket.BucketVisionConstants.BucketCameraConstants;

public class BucketCamera {
    private final BucketCameraConstants camMeta;
    private final BucketCameraIO io;
    private final BucketCameraIOInputsAutoLogged inputs = new BucketCameraIOInputsAutoLogged();

    private final Alert notConnectedAlert;

    public BucketCamera(BucketCameraConstants camMeta, BucketCameraIO io) {
        this.camMeta = camMeta;
        this.io = io;

        notConnectedAlert = new Alert("Bucket camera \"" + camMeta.hardwareName + "\" is not connected", AlertType.kError);
    }

    public Optional<BucketCameraResult> periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/BucketVision/" + camMeta.hardwareName, inputs);

        notConnectedAlert.set(!inputs.isConnected);
        return BucketCameraResult.from(camMeta, inputs);
    }

    public static class BucketCameraResult {
        public final BucketCameraConstants camMeta;
        public final BucketCameraTarget[] targets;

        private BucketCameraResult(BucketCameraConstants camMeta, BucketCameraTarget[] targets) {
            this.camMeta = camMeta;
            this.targets = targets;
        }

        public static Optional<BucketCameraResult> from(BucketCameraConstants camMeta, BucketCameraIOInputs inputs) {
            if (!inputs.isConnected || inputs.targets.length <= 0) return Optional.empty();

            return Optional.of(new BucketCameraResult(
                camMeta,
                inputs.targets
            ));
        }
    }
}
