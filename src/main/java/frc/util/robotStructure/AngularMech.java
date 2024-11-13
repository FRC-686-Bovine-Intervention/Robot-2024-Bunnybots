package frc.util.robotStructure;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;

public class AngularMech extends Mechanism3d {
    private Transform3d angle;

    private final Vector<N3> axis;
    public void setAngle(Angle angle) {
        this.angle = new Transform3d(Translation3d.kZero, new Rotation3d(axis, angle));
    }

    public AngularMech(Transform3d base, Vector<N3> axis) {
        super(base);
        this.axis = axis;
    }

    @Override
    public Transform3d getRobotRelative() {
        return super.getRobotRelative().plus(angle);
    }
    @Override
    public Pose3d getFieldRelative() {
        return super.getFieldRelative().plus(angle);
    }
}
