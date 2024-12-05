package frc.robot.subsystems.vision.apriltag;

import java.nio.ByteBuffer;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;

public interface ApriltagCameraIO {

    @AutoLog
    public class ApriltagCameraIOInputs {
        public boolean isConnected;
        public double timestamp;
        public ApriltagCameraTarget[] targets = new ApriltagCameraTarget[0];
        public Pose3d estimatedRobotPose = new Pose3d();
    }

    public default void updateInputs(ApriltagCameraIOInputs inputs) {}

    public static class ApriltagCameraTarget implements StructSerializable {
        public final int tagID;
        public final Transform3d bestCameraToTag;
        public final Transform3d altCameraToTag;
        public final double poseAmbiguity;

        public ApriltagCameraTarget(int tagID, Transform3d bestCameraToTag, Transform3d altCameraToTag, double poseAmbiguity) {
            this.tagID = tagID;
            this.bestCameraToTag = bestCameraToTag;
            this.altCameraToTag = altCameraToTag;
            this.poseAmbiguity = poseAmbiguity;
        }

        public static final ApriltagCameraTargetStruct struct = new ApriltagCameraTargetStruct();
        public static class ApriltagCameraTargetStruct implements Struct<ApriltagCameraTarget> {
            @Override
            public Class<ApriltagCameraTarget> getTypeClass() {
                return ApriltagCameraTarget.class;
            }

            @Override
            public String getTypeName() {
                return "ApriltagTarget";
            }

            @Override
            public int getSize() {
                return kSizeInt32 * 1 + Transform3d.struct.getSize() * 2 + kSizeDouble * 1;
            }

            @Override
            public String getSchema() {
                return "int tagID;Transform3d bestCameraToTag;Transform3d altCameraToTag";
            }

            @Override
            public Struct<?>[] getNested() {
                return new Struct[]{Transform3d.struct,Transform3d.struct};
            }

            @Override
            public ApriltagCameraTarget unpack(ByteBuffer bb) {
                var tagID = bb.getInt();
                var bestCameraToTag = Transform3d.struct.unpack(bb);
                var altCameraToTag = Transform3d.struct.unpack(bb);
                var poseAmbiguity = bb.getDouble();
                return new ApriltagCameraTarget(tagID, bestCameraToTag, altCameraToTag, poseAmbiguity);
            }

            @Override
            public void pack(ByteBuffer bb, ApriltagCameraTarget value) {
                bb.putInt(value.tagID);
                Transform3d.struct.pack(bb, value.bestCameraToTag);
                Transform3d.struct.pack(bb, value.altCameraToTag);
                bb.putDouble(value.poseAmbiguity);
            }
        }
    }
}
