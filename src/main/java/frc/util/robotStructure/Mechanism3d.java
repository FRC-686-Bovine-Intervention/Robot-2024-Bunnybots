package frc.util.robotStructure;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Transform3d;

public abstract class Mechanism3d extends ChildBase {
    public static final String KEY = "Mechanism3d";

    public Mechanism3d(Transform3d base) {
        super(base);
    }

    public void log(String key) {
        Logger.recordOutput(key, getRobotRelative());
    }
}
