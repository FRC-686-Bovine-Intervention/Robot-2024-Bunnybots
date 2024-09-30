package frc.robot.subsystems.vision.apriltag;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.VisionConstants.Camera;
import frc.robot.subsystems.vision.apriltag.ApriltagCameraIO.ApriltagCameraResult;
import frc.robot.util.led.animation.FillAnimation;
import frc.robot.util.led.strips.LEDStrip;

public class ApriltagCamera {
    private final Camera cameraMeta;
    private final ApriltagCameraIO cameraIO;
    private final ApriltagCameraIOInputsAutoLogged inputs = new ApriltagCameraIOInputsAutoLogged();

    public ApriltagCamera(Camera cameraMeta, ApriltagCameraIO cameraIO, LEDStrip connectedStrip) {
        this.cameraMeta = cameraMeta;
        this.cameraIO = cameraIO;
        new Trigger(DriverStation::isDisabled).debounce(1).whileTrue(new FillAnimation(2, () -> (inputs.isConnected ? Color.kGreen : Color.kOrange), connectedStrip));
    }

    public Optional<ApriltagCameraResult> periodic() {
        cameraIO.updateInputs(inputs);
        Logger.processInputs("ApriltagVision/" + cameraMeta.name(), inputs);
        return ApriltagCameraResult.from(cameraMeta, inputs);
    }
}