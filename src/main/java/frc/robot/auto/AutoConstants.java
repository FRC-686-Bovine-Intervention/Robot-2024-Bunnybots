package frc.robot.auto;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Time;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants;
import frc.util.AllianceFlipUtil.FlippedPose2d;

public final class AutoConstants {
    public static final Time allottedAutoTime = Seconds.of(15.3);
    public static final Time disabledTime = Seconds.of(3);

    public static final FlippedPose2d innerStartPose = FlippedPose2d.fromBlue(new Pose2d(
        new Translation2d(
            FieldConstants.highGoalScoreX,
            FieldConstants.stackingGridOuterEdgeY.minus(RobotConstants.centerToSideBumper)
        ),
        Rotation2d.k180deg
    ));
    public static final FlippedPose2d outerStartPose = FlippedPose2d.fromBlue(new Pose2d(
        new Translation2d(
            FieldConstants.stackingGridOuterEdgeX.plus(RobotConstants.centerToFrontBumper),
            FieldConstants.bucket2Y
        ),
        Rotation2d.k180deg
    ));
}
