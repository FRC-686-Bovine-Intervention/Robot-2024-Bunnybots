package frc.util.robotStructure.linear;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import frc.util.robotStructure.Mechanism3d;

public class LinearMech extends Mechanism3d<Distance> {
    public LinearMech(Transform3d base, Vector<N3> axis) {
        super(base, axis);
    }
    public void setData(Distance distance) {
        transform = new Transform3d(new Translation3d(axis.times(distance.in(Meters))), Rotation3d.kZero);
    }
}
