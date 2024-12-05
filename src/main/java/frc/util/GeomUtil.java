package frc.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public class GeomUtil {
    public static Transform3d toTransform3d(Pose3d pose) {
        return new Transform3d(
            pose.getTranslation(),
            pose.getRotation()
        );
    }
}
