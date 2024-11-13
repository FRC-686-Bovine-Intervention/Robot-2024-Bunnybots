package frc.util.robotStructure.angle;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import frc.util.robotStructure.Mechanism3d;

public class AngularMech extends Mechanism3d<Angle> {
    public AngularMech(Transform3d base, Vector<N3> axis) {
        super(base, axis);
    }
    public void setData(Angle angle) {
        transform = new Transform3d(Translation3d.kZero, new Rotation3d(axis, angle));
    }
}
