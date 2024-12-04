package frc.robot.subsystems.vision.bucket;

import java.nio.ByteBuffer;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;

public interface BucketCameraIO {

    @AutoLog
    public class BucketCameraIOInputs {
        public boolean isConnected;
        public BucketCameraTarget[] targets = new BucketCameraTarget[0];
    }

    public default void updateInputs(BucketCameraIOInputs inputs) {}

    public static class BucketCameraTarget implements StructSerializable {
        public final Rotation3d cameraPose;
        public final double area;

        public BucketCameraTarget(Rotation3d cameraPose, double area) {
            this.cameraPose = cameraPose;
            this.area = area;
        }

        public static final BucketCameraTargetStruct struct = new BucketCameraTargetStruct();
        public static class BucketCameraTargetStruct implements Struct<BucketCameraTarget> {
            @Override
            public Class<BucketCameraTarget> getTypeClass() {
                return BucketCameraTarget.class;
            }

            @Override
            public String getTypeName() {
                return "BucketTarget";
            }

            @Override
            public int getSize() {
                return Rotation3d.struct.getSize() * 1 + kSizeDouble * 1;
            }

            @Override
            public String getSchema() {
                return "Rotation3d cameraPose;double area";
            }

            @Override
            public Struct<?>[] getNested() {
                return new Struct[]{Rotation3d.struct};
            }

            @Override
            public BucketCameraTarget unpack(ByteBuffer bb) {
                var cameraPose = Rotation3d.struct.unpack(bb);
                var area = bb.getDouble();
                return new BucketCameraTarget(cameraPose, area);
            }

            @Override
            public void pack(ByteBuffer bb, BucketCameraTarget value) {
                Rotation3d.struct.pack(bb, value.cameraPose);
                bb.putDouble(value.area);
            }
        }
    }
}
