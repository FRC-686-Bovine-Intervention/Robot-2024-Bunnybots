package frc.robot.subsystems.vision.apriltag;

import static edu.wpi.first.units.Units.Meters;

import java.util.Arrays;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.RobotState;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.apriltag.ApriltagCamera.ApriltagCameraResult;
import frc.util.LoggedTunableNumber;
import frc.util.VirtualSubsystem;

public class ApriltagVision extends VirtualSubsystem {
    private final ApriltagCamera[] cameras;

    private static final LoggedTunableNumber ambiguityThreshold = new LoggedTunableNumber("Vision/Apriltags/Filtering/Ambiguity Threshold", 0.4);
    private static final LoggedTunableNumber xyStdDevCoef = new LoggedTunableNumber("Vision/Apriltags/Std Devs/XY Coef", 0.4);
    private static final LoggedTunableNumber thetaStdDevCoef = new LoggedTunableNumber("Vision/Apriltags/Std Devs/Theta Coef", 0.4);

    public ApriltagVision(ApriltagCamera... cameras) {
        System.out.println("[Init ApriltagVision] Instantiating ApriltagVision");
        this.cameras = cameras;
        Logger.recordOutput("Field/Tag Poses", FieldConstants.apriltagLayout.getTags().stream().map((tag) -> tag.pose).toArray(Pose3d[]::new));
        Logger.recordOutput("Field/Tag IDs", FieldConstants.apriltagLayout.getTags().stream().mapToInt((tag) -> tag.ID).toArray());
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Apriltag Cam Poses", Arrays.stream(
            new VisionConstants.ApriltagCameraConstants[]{
                VisionConstants.frontLeftApriltagCamera,
                VisionConstants.frontRightApriltagCamera,
                VisionConstants.backLeftApriltagCamera,
                VisionConstants.backRightApriltagCamera
            })
            .map((constants) -> constants.mount.getFieldRelative())
            .toArray(Pose3d[]::new)
        );
        var results = Arrays.stream(cameras).map(ApriltagCamera::periodic).filter(Optional::isPresent).map(Optional::get).toArray(ApriltagCameraResult[]::new);
        for (var result : results) {
            if (result.targets.length == 0) continue; // Just in case the filtering above doesn't catch no target frames

            var loggingKey = "Vision/Apriltags/Results/" + result.camMeta.hardwareName;

            var usableTags = Arrays
                .stream(result.targets)
                .map((target) -> {
                    var optTagPose = FieldConstants.apriltagLayout.getTagPose(target.tagID);
                    if (optTagPose.isEmpty()) return Optional.empty();
                    return Optional.of(new AprilTag(target.tagID, optTagPose.get()));
                })
                .filter(Optional::isPresent)
                .toArray(AprilTag[]::new)
            ;
            Logger.recordOutput(loggingKey + "/Targets/Tag IDs", Arrays.stream(usableTags).mapToInt((tag) -> tag.ID).toArray());
            Logger.recordOutput(loggingKey + "/Targets/Tag Poses", Arrays.stream(usableTags).map((tag) -> tag.pose).toArray(Pose3d[]::new));

            // final because averageTagDist mapToDouble needs it
            final Pose3d cameraPose3d;
            final Pose3d robotPose3d;
            var useVisionRotation = false;

            if (result.targets.length >= 2) {
                robotPose3d = result.estimatedRobotPose;
                cameraPose3d = robotPose3d.transformBy(result.camMeta.mount.getRobotRelative());
                useVisionRotation = true;
            } else if (result.targets.length == 1) {
                var target = result.targets[0];
                var bestCameraPose = FieldConstants.apriltagLayout.getTagPose(target.tagID).get().transformBy(target.bestCameraToTag.inverse());
                var bestRobotPose = bestCameraPose.transformBy(result.camMeta.mount.getRobotRelative().inverse());
                var altCameraPose = FieldConstants.apriltagLayout.getTagPose(target.tagID).get().transformBy(target.altCameraToTag.inverse());
                var altRobotPose = altCameraPose.transformBy(result.camMeta.mount.getRobotRelative().inverse());
                if (result.targets[0].poseAmbiguity >= ambiguityThreshold.get()) {
                    var currentRotation = RobotState.getInstance().getPose().getRotation();
                    var bestRotation = bestRobotPose.getRotation().toRotation2d();
                    var altRotation = altRobotPose.getRotation().toRotation2d();
                    if (Math.abs(currentRotation.minus(bestRotation).getRadians()) < Math.abs(currentRotation.minus(altRotation).getRadians())) {
                        cameraPose3d = bestCameraPose;
                        robotPose3d = bestRobotPose;
                    } else {
                        cameraPose3d = altCameraPose;
                        robotPose3d = altRobotPose;
                    }
                } else {
                    cameraPose3d = null;
                    robotPose3d = null;
                }
            } else {
                cameraPose3d = null;
                robotPose3d = null;
            }
            var robotPose2d = robotPose3d.toPose2d();
            Logger.recordOutput(loggingKey + "/Poses/Robot2d", robotPose2d);
            Logger.recordOutput(loggingKey + "/Poses/Robot3d", robotPose3d);
            Logger.recordOutput(loggingKey + "/Poses/Camera3d", cameraPose3d);

            // Filtering
            var inField = VisionConstants.acceptableFieldBox.withinBounds(robotPose2d.getTranslation());
            var closeToFloor = robotPose3d.getTranslation().getMeasureZ().isNear(Meters.zero(), VisionConstants.zMargin);

            Logger.recordOutput(loggingKey + "/Filtering/In Field", inField);
            Logger.recordOutput(loggingKey + "/Filtering/Close to Floor", closeToFloor);

            if (
                !inField
                || !closeToFloor
            ) {
                continue;
            }

            // Std Devs
            var optAverageTagDistance = Arrays
                .stream(usableTags)
                .mapToDouble((tag) -> tag.pose.getTranslation().getDistance(cameraPose3d.getTranslation()))
                .average()
            ;
            if (optAverageTagDistance.isEmpty()) continue;
            var averageTagDistance = optAverageTagDistance.getAsDouble();
            Logger.recordOutput(loggingKey + "/Std Devs/Average Distance", averageTagDistance);

            double xyStdDev =
                xyStdDevCoef.get()
                * averageTagDistance * averageTagDistance
                / usableTags.length
                * result.camMeta.cameraStdCoef
            ;
            double thetaStdDev =
                (useVisionRotation) ? (
                    thetaStdDevCoef.get()
                    * averageTagDistance * averageTagDistance
                    / usableTags.length
                    * result.camMeta.cameraStdCoef
                ) : (
                    Double.POSITIVE_INFINITY
                )
            ;
            Logger.recordOutput(loggingKey + "/Std Devs/XY", xyStdDev);
            Logger.recordOutput(loggingKey + "/Std Devs/Theta", thetaStdDev);

            RobotState.getInstance().addVisionMeasurement(
                robotPose2d,
                VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev),
                result.timestamp
            );
        }
    }

    // public static record ApriltagResultTests(
    //     boolean inField,
    //     boolean closeToFloor
    // ) implements StructSerializable {
        
    //     public static ApriltagResultTests runTests(ApriltagCameraResult result) {
    //         var inField = false;
    //         var closeToFloor = false;
    //         return new ApriltagResultTests(
    //             inField,
    //             closeToFloor
    //         );
    //     }

    //     public boolean allGood() {
    //         return inField && closeToFloor;
    //     }

    //     public static final ApriltagResultTestsStruct struct = new ApriltagResultTestsStruct();
    //     public static class ApriltagResultTestsStruct implements Struct<ApriltagResultTests> {
    //         @Override
    //         public Class<ApriltagResultTests> getTypeClass() {
    //             return ApriltagResultTests.class;
    //         }

    //         @Override
    //         public String getTypeName() {
    //             return "ApriltagResultTests";
    //         }

    //         @Override
    //         public int getSize() {
    //             return kSizeInt8 * 1;
    //         }

    //         @Override
    //         public String getSchema() {
    //             return "boolean inField;boolean closeToFloor";
    //         }

    //         @Override
    //         public ApriltagResultTests unpack(ByteBuffer bb) {
    //             // for (boolean b : new boolean[]{
    //             //     value.inField,
    //             //     value.closeToFloor
    //             // }) {
    //             //     val <<= 1;
    //             //     if (b) {
    //             //         val |= 1;
    //             //     }
    //             // }
    //             // return new ApriltagResultTests(
    //             //     bb.get
    //             // );
    //             return new ApriltagResultTests(false, false);
    //         }

    //         @Override
    //         public void pack(ByteBuffer bb, ApriltagResultTests value) {
    //             byte val = 0;
    //             for (boolean b : new boolean[]{
    //                 value.inField,
    //                 value.closeToFloor
    //             }) {
    //                 val <<= 1;
    //                 if (b) {
    //                     val |= 1;
    //                 }
    //             }
    //             bb.put(val);
    //         }
    //     }
    // }
}