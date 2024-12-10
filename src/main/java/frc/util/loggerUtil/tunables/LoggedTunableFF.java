package frc.util.loggerUtil.tunables;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.SlotConfigs;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.util.LoggedTunableNumber;

public class LoggedTunableFF {
    private final LoggedTunableNumber kS;
    private final LoggedTunableNumber kG;
    private final LoggedTunableNumber kV;
    private final LoggedTunableNumber kA;

    public LoggedTunableFF(String key, double kS, double kG, double kV, double kA) {
        this.kS = new LoggedTunableNumber(key + "/kS", kS);
        this.kG = new LoggedTunableNumber(key + "/kG", kG);
        this.kV = new LoggedTunableNumber(key + "/kV", kV);
        this.kA = new LoggedTunableNumber(key + "/kA", kA);
    }

    public boolean hasChanged(int hashCode) {
        return LoggedTunableNumber.hasChanged(hashCode, kS, kG, kV, kA);
    }

    public TrapezoidProfile getTrapezoidProfile() {
        return new TrapezoidProfile(
            new Constraints(
                kV.get(),
                kA.get()
            )
        );
    }

    public void update(SlotConfigs ff) {
        ff
            .withKS(kS.get())
            .withKG(kG.get())
            .withKV(kV.get())
            .withKA(kA.get())
        ;
    }
    public void update(Slot0Configs ff) {
        ff
            .withKS(kS.get())
            .withKG(kG.get())
            .withKV(kV.get())
            .withKA(kA.get())
        ;
    }
    public void update(Slot1Configs ff) {
        ff
            .withKS(kS.get())
            .withKG(kG.get())
            .withKV(kV.get())
            .withKA(kA.get())
        ;
    }
    public void update(Slot2Configs ff) {
        ff
            .withKS(kS.get())
            .withKG(kG.get())
            .withKV(kV.get())
            .withKA(kA.get())
        ;
    }
}
