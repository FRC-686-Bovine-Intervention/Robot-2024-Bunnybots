package frc.util.loggerUtil.tunables;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.SlotConfigs;

import edu.wpi.first.math.controller.PIDController;
import frc.util.wpilib.ProfiledPIDController;

public class LoggedTunablePID {
    private final LoggedTunableNumber kP;
    private final LoggedTunableNumber kI;
    private final LoggedTunableNumber kD;

    public LoggedTunablePID(String key, double kP, double kI, double kD) {
        this.kP = new LoggedTunableNumber(key + "/kP", kP);
        this.kI = new LoggedTunableNumber(key + "/kI", kI);
        this.kD = new LoggedTunableNumber(key + "/kD", kD);
    }

    public boolean hasChanged(int hashCode) {
        return LoggedTunableNumber.hasChanged(hashCode, kP, kI, kD);
    }

    public void update(PIDController pid) {
        pid.setPID(
            kP.get(),
            kI.get(),
            kD.get()
        );
    }
    public void update(ProfiledPIDController pid) {
        pid.setPID(
            kP.get(),
            kI.get(),
            kD.get()
        );
    }
    public void update(SlotConfigs pid) {
        pid
            .withKP(kP.get())
            .withKI(kI.get())
            .withKD(kD.get())
        ;
    }
    public void update(Slot0Configs pid) {
        pid
            .withKP(kP.get())
            .withKI(kI.get())
            .withKD(kD.get())
        ;
    }
    public void update(Slot1Configs pid) {
        pid
            .withKP(kP.get())
            .withKI(kI.get())
            .withKD(kD.get())
        ;
    }
    public void update(Slot2Configs pid) {
        pid
            .withKP(kP.get())
            .withKI(kI.get())
            .withKD(kD.get())
        ;
    }
}
