package frc.util.robotStructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public class Root implements Component {
    private Pose3d pose;

    public void setPose(Pose2d pose) {
        this.pose = new Pose3d(pose);
    }

    @Override
    public Transform3d getRobotRelative() {
        return Transform3d.kZero;
    }

    @Override
    public Pose3d getFieldRelative() {
        return pose;
    }
}
