package frc.util.robotStructure;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;

public class LinearMech extends Mechanism3d {
    private Transform3d linear;

    private final Vector<N3> axis;
    public void setDistance(Distance distance) {
        this.linear = new Transform3d(new Translation3d(axis.times(distance.in(Meters))), Rotation3d.kZero);
    }

    public LinearMech(Transform3d base, Vector<N3> axis) {
        super(base);
        this.axis = axis;
    }

    @Override
    public Transform3d getRobotRelative() {
        return super.getRobotRelative().plus(linear);
    }
    @Override
    public Pose3d getFieldRelative() {
        return super.getFieldRelative().plus(linear);
    }
}
