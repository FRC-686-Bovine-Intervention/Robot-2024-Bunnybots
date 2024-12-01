package frc.util.loggerUtil;

import com.ctre.phoenix6.configs.SlotConfigs;

import edu.wpi.first.math.controller.PIDController;
import frc.util.LoggedTunableNumber;
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
}
