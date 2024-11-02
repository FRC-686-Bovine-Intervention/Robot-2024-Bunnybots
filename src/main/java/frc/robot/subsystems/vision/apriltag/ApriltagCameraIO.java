package frc.robot.subsystems.vision.apriltag;

import java.util.Arrays;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants.VisionConstants.Camera;

public interface ApriltagCameraIO {

    @AutoLog
    public class ApriltagCameraIOInputs {
        public boolean isConnected;
        public boolean hasResult;
        public double timestamp;
        public int[] tagsSeen = new int[0];
        public double[] cameraToTagDist = new double[0];
        public Pose3d estimatedRobotPose = new Pose3d();

        // public Optional<ApriltagCameraResult> getResult() {
        //     return Optional.ofNullable(hasResult ? result : null);
        // }

        // public Optional<Pose3d> getPose() {
        //     return getResult().map((r) -> r.estimatedRobotPose);
        // }
    }

    public static class ApriltagCameraResult /* implements StructSerializable */ {
        public final double timestamp;
        public final int[] tagsSeen;
        public final double[] cameraToTagDist;
        public final Pose3d estimatedRobotPose;
        public final Camera cameraMeta;

        public ApriltagCameraResult(Camera cameraMeta, double timestamp, int[] tagsSeen, double[] cameraToTagDist, Pose3d estimatedRobotPose) {
            this.cameraMeta = cameraMeta;
            this.timestamp = timestamp;
            this.tagsSeen = tagsSeen;
            this.cameraToTagDist = cameraToTagDist;
            this.estimatedRobotPose = estimatedRobotPose;
        }

        public static Optional<ApriltagCameraResult> from(Camera meta, ApriltagCameraIOInputs inputs) {
            if(!inputs.hasResult) return Optional.empty();
            return Optional.of(new ApriltagCameraResult(
                meta,
                inputs.timestamp,
                inputs.tagsSeen,
                inputs.cameraToTagDist,
                inputs.estimatedRobotPose
            ));
        }

        public double getAverageDist() {
            return Arrays.stream(cameraToTagDist).average().orElse(0);
        }

    //     public static final ApriltagCameraResultStruct struct = new ApriltagCameraResultStruct();
    //     public static class ApriltagCameraResultStruct implements Struct<ApriltagCameraResult> {
    //         @Override
    //         public Class<ApriltagCameraResult> getTypeClass() {
    //             return ApriltagCameraResult.class;
    //         }

    //         @Override
    //         public String getTypeString() {
    //             return "struct:ApriltagCameraResult";
    //         }

    //         @Override
    //         public int getSize() {
    //             return kSizeDouble * 2 + Pose3d.struct.getSize();
    //         }

    //         @Override
    //         public String getSchema() {
    //             return "double timestamp;double cameraToTargetDist;Pose3d estimatedRobotPose";
    //         }

    //         @Override
    //         public Struct<?>[] getNested() {
    //             return new Struct<?>[]{Pose3d.struct};
    //         }

    //         @Override
    //         public ApriltagCameraResult unpack(ByteBuffer bb) {
    //             var timestamp = bb.getDouble();
    //             var cameraToTargetDist = bb.getDouble();
    //             var estimatedRobotPose = Pose3d.struct.unpack(bb);
    //             return new ApriltagCameraResult(timestamp, cameraToTargetDist, estimatedRobotPose);
    //         }

    //         @Override
    //         public void pack(ByteBuffer bb, ApriltagCameraResult value) {
    //             bb.putDouble(value.timestamp);
    //             bb.putDouble(value.cameraToTargetDist);
    //             Pose3d.struct.pack(bb, value.estimatedRobotPose);
    //         }
    //     }
    }
    
    public default void updateInputs(ApriltagCameraIOInputs inputs) {}
}
