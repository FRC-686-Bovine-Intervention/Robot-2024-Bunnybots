package frc.util.loggerUtil.tunables;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Measure;
import frc.util.LoggedTunableMeasure;

public class LoggedTunableLinearProfile {
    private final LoggedTunableMeasure<LinearVelocityUnit> kV;
    private final LoggedTunableMeasure<LinearAccelerationUnit> kA;

    public LoggedTunableLinearProfile(String key,
        Measure<LinearVelocityUnit> kV, Measure<LinearAccelerationUnit> kA
    ) {
        this.kV = new LoggedTunableMeasure<>(key + "/kV", kV);
        this.kA = new LoggedTunableMeasure<>(key + "/kA", kA);
    }

    public boolean hasChanged(int hashCode) {
        return LoggedTunableMeasure.hasChanged(hashCode, kV, kA);
    }

    public TrapezoidProfile getTrapezoidProfile() {
        return new TrapezoidProfile(
            new Constraints(
                kV.get(),
                kA.get()
            )
        );
    }

    public void update(ProfiledPIDController profiledPIDController) {
        profiledPIDController.setConstraints(
            new Constraints(
                kV.get(),
                kA.get()
            )
        );
    }
}
