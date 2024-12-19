package frc.robot.subsystems.vision.apriltag;

import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.RobotState;
import frc.robot.constants.FieldConstants;

public class ApriltagVisionSim extends ApriltagVision {
    private final ApriltagCameraSim[] cameras;
    private VisionSystemSim visionSim = new VisionSystemSim("apriltag");

    public ApriltagVisionSim(ApriltagCameraSim... cameras) {
        super(cameras);
        this.cameras = cameras;
        visionSim.addAprilTags(FieldConstants.apriltagLayout);

        for (var cam : this.cameras) {
            visionSim.addCamera(cam.getCameraSim(), cam.getRobotRelative());
        }
    }

    public void updateRobotPos(Pose2d robotPose) {
        visionSim.update(robotPose);
    }

    public void updateCamPos() {
        for (var cam : this.cameras) {
            visionSim.adjustCamera(cam.getCameraSim(), cam.getRobotRelative());
        }
    }

    @Override
    public void periodic() {
        updateRobotPos(RobotState.getInstance().getPose());
        updateCamPos();
        super.periodic();
    }
}
