package frc.robot.constants;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Filesystem;
import frc.util.AllianceFlipUtil.FlippedPose2d;

public final class FieldConstants {
    public static final Distance fieldLength = Inches.of(648);
    public static final Distance fieldWidth =  Inches.of(324);

    public static final AprilTagFieldLayout apriltagLayout;
    static {
        AprilTagFieldLayout a = null;
        try {
            a = new AprilTagFieldLayout(Filesystem.getDeployDirectory() + "/Bunnybots2024ApriltagLayout.json");
        } catch(Exception e) {
            e.printStackTrace();
        }
        apriltagLayout = a;
    }

    public static final Distance bucketRadius = Inches.of(6);
    public static final Distance bucketHeight = Inches.of(14.5);

    public static final Distance bucket3Y = Inches.of(227.778477);
    public static final Distance bucket2Y = Inches.of(262.168859);
    public static final Distance bucket1Y = Inches.of(300.114098);

    public static final Distance stackingGridOuterEdgeX = Inches.of(27.5);
    public static final Distance stackingGridOuterEdgeY = Inches.of(182.5);
    public static final Distance stackingGridInnerEdgeY = Inches.of(186);
    public static final Distance denWallInnerEdgeY = Inches.of(102.5);
    public static final Distance yardBottomOuterEdgeY = Inches.of(146.5);
    public static final Distance yardTopOuterEdgeY = Inches.of(201.5);
    public static final Distance yardRightOuterEdgeX = Inches.of(203.09);
    public static final Distance yardLeftOuterEdgeX = Inches.of(148.09);
    public static final Distance topObsticalBottomEdgeY = Inches.of(260);
    public static final Distance topObsticalTopEdgeY = Inches.of(284);

    // Den Entry
    public static final Distance denPreEntryX = yardRightOuterEdgeX.plus(RobotConstants.centerToFrontBumper);
    public static final Distance denEntryX = yardLeftOuterEdgeX.minus(RobotConstants.centerToFrontBumper);
    // | Source Entry
    public static final Distance denSourceEntryY = denWallInnerEdgeY.plus(yardBottomOuterEdgeY).divide(2);
    public static final FlippedPose2d denSourcePreEntry = FlippedPose2d.fromBlue(new Pose2d(
        new Translation2d(
            denPreEntryX,
            denSourceEntryY
        ),
        Rotation2d.k180deg
    ));
    public static final FlippedPose2d denSourceEntry = FlippedPose2d.fromBlue(new Pose2d(
        new Translation2d(
            denEntryX,
            denSourceEntryY
        ),
        Rotation2d.k180deg
    ));
    // | Field Entry
    public static final Distance denFieldEntryY = yardTopOuterEdgeY.plus(topObsticalBottomEdgeY).divide(2);
    public static final FlippedPose2d denFieldPreEntry = FlippedPose2d.fromBlue(new Pose2d(
        new Translation2d(
            denPreEntryX,
            denFieldEntryY
        ),
        Rotation2d.k180deg
    ));
    public static final FlippedPose2d denFieldEntry = FlippedPose2d.fromBlue(new Pose2d(
        new Translation2d(
            denEntryX,
            denFieldEntryY
        ),
        Rotation2d.k180deg
    ));
    public static final Distance denEntryDecisionY = denSourceEntryY.plus(denFieldEntryY).divide(2);

    // High Goal Score
    public static final Distance highGoalScoreX = Inches.of(7.5).plus(RobotConstants.centerToFrontBumper);
    public static final Distance highGoalPreScoreX = highGoalScoreX.plus(Feet.one());
    // | Stacking Side
    public static final Distance highGoalStackingSideY = Inches.of(155.5);
    public static final FlippedPose2d highGoalScoreStackingSide = FlippedPose2d.fromBlue(new Pose2d(
        new Translation2d(
            highGoalScoreX,
            highGoalStackingSideY
        ),
        Rotation2d.k180deg
    ));
    public static final FlippedPose2d highGoalPreScoreStackingSide = FlippedPose2d.fromBlue(new Pose2d(
        new Translation2d(
            highGoalPreScoreX,
            highGoalStackingSideY
        ),
        Rotation2d.k180deg
    ));
    // | Source Side
    public static final Distance highGoalSourceSideY = Inches.of(129.5);
    public static final FlippedPose2d highGoalScoreSourceSide = FlippedPose2d.fromBlue(new Pose2d(
        new Translation2d(
            highGoalScoreX,
            highGoalSourceSideY
        ),
        Rotation2d.k180deg
    ));
    public static final FlippedPose2d highGoalPreScoreSourceSide = FlippedPose2d.fromBlue(new Pose2d(
        new Translation2d(
            highGoalPreScoreX,
            highGoalSourceSideY
        ),
        Rotation2d.k180deg
    ));

    // Stacking Score
    public static final Distance stackingScoreX = stackingGridOuterEdgeX.plus(RobotConstants.centerToFrontBumper);
    public static final Distance stackingMinY = stackingGridInnerEdgeY.plus(bucketRadius);
    public static final Distance stackingMaxY = fieldWidth.minus(RobotConstants.centerToSideBumper);
    
    // Yard Score
    public static final FlippedPose2d yardFieldScore = FlippedPose2d.fromRed(new Pose2d(
        new Translation2d(
            yardRightOuterEdgeX.plus(RobotConstants.centerToFrontBumper),
            yardBottomOuterEdgeY.plus(yardTopOuterEdgeY).divide(2)
        ),
        Rotation2d.k180deg
    ));
}
