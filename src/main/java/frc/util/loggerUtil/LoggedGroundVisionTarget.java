package frc.util.loggerUtil;

import java.nio.ByteBuffer;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.subsystems.vision.VisionConstants.Camera;
import frc.util.MathExtraUtil;

public class LoggedGroundVisionTarget implements StructSerializable {
    public Translation2d robotRelativePos;
    public double confidence;

    public LoggedGroundVisionTarget() {
        this(null, 0);
    }

    public LoggedGroundVisionTarget(Translation2d posRobotRelative, double confidence) {
        this.robotRelativePos = posRobotRelative;
        this.confidence = confidence;
    }

    public LoggedGroundVisionTarget updateFrom(Camera camera, PhotonTrackedTarget target) {
        var robotToCam = camera.getRobotToCam();
        var yaw = target.getYaw();

        var yBottom = MathExtraUtil.average(target.getDetectedCorners().get(0).y, target.getDetectedCorners().get(1).y);
        var imageHeight = camera.imageSize.getY();
        var normalized = (imageHeight / 2.0 - yBottom) / imageHeight;
        var pitch = normalized * camera.fov.getY();

        var distOut = robotToCam.getZ() / Math.tan(pitch);
        var distOff = distOut * Math.tan(yaw);
        var camToTargetTranslation = new Translation3d(distOut, distOff, -robotToCam.getZ());

        var transform = robotToCam.plus(new Transform3d(camToTargetTranslation, Rotation3d.kZero));
        this.robotRelativePos = new Pose3d().transformBy(transform).toPose2d().getTranslation();
        
        return this;
    }

    public LoggedGroundVisionTarget withConfidence(double confidence) {
        this.confidence = confidence;
        return this;
    }

    public static final LoggedGroundVisionTargetStruct struct = new LoggedGroundVisionTargetStruct();
    public static class LoggedGroundVisionTargetStruct implements Struct<LoggedGroundVisionTarget> {
        @Override
        public Class<LoggedGroundVisionTarget> getTypeClass() {
            return LoggedGroundVisionTarget.class;
        }

        @Override
        public String getTypeName() {
            return "struct:LoggedGroundVisionTarget";
        }

        @Override
        public int getSize() {
            return Translation2d.struct.getSize() + kSizeDouble;
        }

        @Override
        public String getSchema() {
            return "Translation2d posRobotRelative;double confidence;";
        }

        @Override
        public Struct<?>[] getNested() {
            return new Struct<?>[] {Translation2d.struct};
        }

        @Override
        public LoggedGroundVisionTarget unpack(ByteBuffer bb) {
            var posRobotRelative = Translation2d.struct.unpack(bb);
            var confidence = bb.getDouble();
            return new LoggedGroundVisionTarget(posRobotRelative, confidence);
        }

        @Override
        public void pack(ByteBuffer bb, LoggedGroundVisionTarget value) {
            Translation2d.struct.pack(bb, value.robotRelativePos);
            bb.putDouble(value.confidence);
        }
    }
}
