package frc.util;

import static edu.wpi.first.units.Units.Celsius;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.TemperatureUnit;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class TalonFXTempAlerts {
    private final TalonFX motor;
    private final Measure<TemperatureUnit> warningThreshold;
    private final Alert warning;
    private final Alert error;

    public TalonFXTempAlerts(TalonFX motor, String name) {
        this(motor, name, Celsius.of(70));
    }
    public TalonFXTempAlerts(TalonFX motor, String name, Measure<TemperatureUnit> warningThreshold) {
        this.motor = motor;
        this.warningThreshold = warningThreshold;
        this.warning = new Alert(name + " has exceeded " + this.warningThreshold.toShortString(), AlertType.kWarning);
        this.error = new Alert(name + " has shutdown due to high temp", AlertType.kError);
    }

    public void update() {
        warning.set(motor.getDeviceTemp().getValue().gte(warningThreshold));
        error.set(motor.getFault_DeviceTemp().getValue());
    }
}
