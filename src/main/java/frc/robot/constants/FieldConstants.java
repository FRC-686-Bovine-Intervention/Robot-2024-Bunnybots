package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.units.measure.Distance;

public final class FieldConstants {
    public static final Distance fieldLength = Inches.of(648);
    public static final Distance fieldWidth =  Inches.of(324);

    public static final AprilTagFieldLayout apriltagLayout;
    static {
        AprilTagFieldLayout a = null;
        try {
            a = AprilTagFieldLayout.loadFromResource("Bunnybots2024ApriltagLayout");
        } catch(Exception e) {}
        apriltagLayout = a;
    }

    // public static final FlippedTranslation3d speakerAimPoint = FlippedTranslation3d.fromBlue(new Translation3d(0.240581, 5.547755, 2));
    // public static final FlippedTranslation3d passAimPoint =    FlippedTranslation3d.fromBlue(speakerAimPoint.getBlue().interpolate(new Translation3d(1.83,7.61,2), 0.7));

    // public static final FlippedPose2d subwooferFront =  FlippedPose2d.fromBlue(new Pose2d(new Translation2d(1.45, 5.55), Rotation2d.fromDegrees(+180)));
    // public static final FlippedPose2d subwooferAmp =    FlippedPose2d.fromBlue(new Pose2d(new Translation2d(0.71, 6.72), Rotation2d.fromDegrees(-120)));
    // public static final FlippedPose2d subwooferSource = FlippedPose2d.fromBlue(new Pose2d(new Translation2d(0.71, 6.72), Rotation2d.fromDegrees(+120)));
    
    // public static final FlippedPose2d amp =    FlippedPose2d.fromBlue(new Pose2d(new Translation2d(1.83, 7.61), Rotation2d.fromDegrees(-90)));
    // public static final FlippedPose2d podium = FlippedPose2d.fromBlue(new Pose2d(new Translation2d(2.76, 4.44), Rotation2d.fromDegrees(+157.47)));

    // public static final FlippedPose2d pathfindSpeaker = FlippedPose2d.fromBlue(new Pose2d(new Translation2d(3.45, 5.55), Rotation2d.fromDegrees(+180)));
    // public static final FlippedPose2d pathfindSource =  FlippedPose2d.fromBlue(new Pose2d(new Translation2d(13.41, 1.54), Rotation2d.fromDegrees(+180)));

    // public static final double podiumToSpeakerDist =    speakerAimPoint.getBlue().toTranslation2d().getDistance(podium.getBlue().getTranslation());
    // public static final double subwooferToSpeakerDist = speakerAimPoint.getBlue().toTranslation2d().getDistance(subwooferFront.getBlue().getTranslation());
}
