package frc.robot.subsystems.vision.canister;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.subsystems.vision.VisionConstants.Camera;
import frc.util.loggerUtil.LoggedGroundVisionTarget;

public class CanisterVisionIOPhotonVision implements CanisterVisionIO {
    private final PhotonCamera camera;
    private final Camera cameraMeta;

    private final Alert notConnectedAlert;

    public CanisterVisionIOPhotonVision(Camera camera) {
        this.camera = new PhotonCamera(camera.hardwareName);
        this.cameraMeta = camera;
        notConnectedAlert = new Alert(cameraMeta.hardwareName + " is not connected", AlertType.kError);
    }

    @Override
    public void updateInputs(CanisterVisionIOInputs inputs) {
        inputs.connected = camera.isConnected();
        notConnectedAlert.set(!inputs.connected);
        cameraMeta.connectedConsumer.accept(inputs.connected);
        inputs.trackedCanisters = new LoggedGroundVisionTarget[0];
        if (!inputs.connected) return;
        inputs.trackedCanisters = camera.getLatestResult().getTargets().stream()
            .map((target) -> new LoggedGroundVisionTarget()
                .updateFrom(cameraMeta, target)
                .withConfidence(Math.sqrt(target.getArea()) * CanisterVisionConstants.confidencePerAreaPercent.get()))
            .toArray(LoggedGroundVisionTarget[]::new);
    }
}
