package frc.util.robotStructure;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public interface Component {
    public Transform3d getRobotRelative();
    public Pose3d getFieldRelative();
}
