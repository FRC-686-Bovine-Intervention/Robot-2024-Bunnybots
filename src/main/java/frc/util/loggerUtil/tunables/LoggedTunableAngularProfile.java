package frc.util.loggerUtil.tunables;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.configs.MotionMagicConfigs;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Measure;

public class LoggedTunableAngularProfile {
    private final LoggedTunableMeasure<AngularVelocityUnit> kV;
    private final LoggedTunableMeasure<AngularAccelerationUnit> kA;

    public LoggedTunableAngularProfile(String key,
        Measure<AngularVelocityUnit> kV, Measure<AngularAccelerationUnit> kA
    ) {
        this.kV = new LoggedTunableMeasure<>(key + "/kV", kV);
        this.kA = new LoggedTunableMeasure<>(key + "/kA", kA);
    }

    public boolean hasChanged(int hashCode) {
        return LoggedTunableMeasure.hasChanged(hashCode, kV, kA);
    }

    public void update(MotionMagicConfigs motionMagicConfigs) {
        if(LoggedTunableMeasure.hasChanged(hashCode(), kV, kA)) {
            motionMagicConfigs
                .withMotionMagicCruiseVelocity(kV.in(RotationsPerSecond))
                .withMotionMagicAcceleration(kA.in(RotationsPerSecondPerSecond))
            ;
        }
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
        if(LoggedTunableMeasure.hasChanged(hashCode(), kV, kA)) {
            profiledPIDController.setConstraints(
                new Constraints(
                    kV.get(),
                    kA.get()
                )
            );
        }
    }
}
