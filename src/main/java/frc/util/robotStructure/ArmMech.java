package frc.util.robotStructure;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;

public class ArmMech extends Mechanism3d {
    private Transform3d angle;

    public void setAngle(Angle angle) {
        this.angle = new Transform3d(Translation3d.kZero, new Rotation3d(Radians.zero(), angle, Radians.zero()));
    }

    public ArmMech(Transform3d base) {
        super(base);
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
