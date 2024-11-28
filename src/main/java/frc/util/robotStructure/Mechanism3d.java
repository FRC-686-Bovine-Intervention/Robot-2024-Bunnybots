package frc.util.robotStructure;

import java.util.Arrays;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

public abstract class Mechanism3d<U extends Unit> extends ChildBase {
    public static final String KEY = "Mechanism3d";
    protected final Vector<N3> axis;
    protected Transform3d transform = Transform3d.kZero;

    private static Mechanism3d<?>[] mechanisms;
    public static void registerMechs(Mechanism3d<?>... mechs) {
        mechanisms = mechs;
    }
    public static void logAscopeComponents() {
        Logger.recordOutput(KEY + "/Mechs", Arrays.stream(mechanisms).map(Mechanism3d::getRobotRelative).toArray(Transform3d[]::new));
    }

    public Mechanism3d(Transform3d base, Vector<N3> axis) {
        super(base);
        this.axis = axis;
    }

    public abstract void set(Measure<U> data);

    @Override
    public Transform3d getRobotRelative() {
        return super.getRobotRelative().plus(transform);
    }
    @Override
    public Pose3d getFieldRelative() {
        return super.getFieldRelative().plus(transform);
    }

    public void log(String key) {
        Logger.recordOutput(key, getRobotRelative());
    }
}
