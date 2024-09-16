package frc.robot.subsystems.vision.apriltag;

import java.util.Arrays;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotState;
import frc.robot.subsystems.vision.apriltag.ApriltagCameraIO.ApriltagCameraResult;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.VirtualSubsystem;

public class ApriltagVision extends VirtualSubsystem {

    private final ApriltagCamera[] cameras;

    public ApriltagVision(ApriltagCamera... cameras) {
        System.out.println("[Init ApriltagVision] Instantiating ApriltagVision");
        this.cameras = cameras;
    }

    // private static final LoggedTunableNumber rejectDist = new LoggedTunableNumber("Vision/Apriltags/Reject Distance", 4);
    private final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

    @Override
    public void periodic() {
        var results = Arrays.stream(cameras).map(ApriltagCamera::periodic).filter(Optional::isPresent).map(Optional::get).toArray(ApriltagCameraResult[]::new);
        var accepted = Arrays.stream(results).filter(ApriltagVision::trustResult).toArray(ApriltagCameraResult[]::new);
        var rejected = Arrays.stream(results).filter((r) -> !trustResult(r)).toArray(ApriltagCameraResult[]::new);
        var tagsSeen = Arrays.stream(results).flatMapToInt((r) -> Arrays.stream(r.tagsSeen)).toArray();
        Logger.recordOutput("Vision/Apriltags/Tags Seen", tagsSeen);
        Logger.recordOutput("Vision/Apriltags/Tags Seen Poses", Arrays.stream(tagsSeen).mapToObj(fieldLayout::getTagPose).filter(Optional::isPresent).map(Optional::get).toArray(Pose3d[]::new));
        Logger.recordOutput("Vision/Apriltags/Accepted Poses", Arrays.stream(accepted).map((r) -> r.estimatedRobotPose).toArray(Pose3d[]::new));
        Logger.recordOutput("Vision/Apriltags/Rejected Poses", Arrays.stream(rejected).map((r) -> r.estimatedRobotPose).toArray(Pose3d[]::new));
        Arrays.stream(accepted).forEach((r) -> 
            RobotState.getInstance().addVisionMeasurement(
                r.estimatedRobotPose.toPose2d(),
                computeStdDevs(r),
                r.timestamp
            )
        );
    }

    private static boolean trustResult(ApriltagCameraResult result) {
        return result.getAverageDist() < result.cameraMeta.trustDistance && result.tagsSeen.length >= 2;
    }

    private static final LoggedTunableNumber kTransA = new LoggedTunableNumber("Vision/Apriltags/StdDevs/Translational/aCoef", 2);
    private static final LoggedTunableNumber kTransC = new LoggedTunableNumber("Vision/Apriltags/StdDevs/Translational/cCoef", -0.5);
    private static final LoggedTunableNumber kRotA = new LoggedTunableNumber("Vision/Apriltags/StdDevs/Rotational/aCoef", 5);
    private static final LoggedTunableNumber kRotC = new LoggedTunableNumber("Vision/Apriltags/StdDevs/Rotational/cCoef", 1000);
    private static final LoggedTunableNumber kRotCDisabled = new LoggedTunableNumber("Vision/Apriltags/StdDevs/Rotational/disabledcCoef", 5);
    private static final LoggedTunableNumber kMultiTag = new LoggedTunableNumber("Vision/Apriltags/MultiStdDevs", 0.1);

    private Matrix<N3, N1> computeStdDevs(ApriltagCameraResult result) {
        var averageDist = result.getAverageDist();
        var numTags = result.tagsSeen.length;
        double rotStdDev = (kRotA.get() * averageDist * averageDist + (DriverStation.isEnabled() ? kRotC.get() : kRotCDisabled.get())) / numTags;
        double transStdDev = (kTransA.get() * averageDist * averageDist + kTransC.get()) / numTags * result.cameraMeta.cameraStdCoef;
        if(DriverStation.isAutonomousEnabled()) {
            return VecBuilder.fill(transStdDev, transStdDev, rotStdDev);
        }
        if(numTags >= 2) {
            transStdDev = kMultiTag.get() / numTags * result.cameraMeta.cameraStdCoef;
        }
        return VecBuilder.fill(transStdDev, transStdDev, rotStdDev);
    }
}