package frc.util.robotStructure;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public abstract class ChildBase implements Component {
    private final Transform3d base;
    private Component parent;

    public ChildBase(Transform3d base) {
        this.base = base;
    }

    public ChildBase setParent(Component parent) {
        this.parent = parent;
        return this;
    }

    @Override
    public Transform3d getRobotRelative() {
        return parent.getRobotRelative().plus(base);
    }

    @Override
    public Pose3d getFieldRelative() {
        return parent.getFieldRelative().plus(base);
    }
}
