package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.constants.HardwareDevices;

public class IntakeIOTalon implements IntakeIO {
    protected final TalonSRX motor = HardwareDevices.intakeMotorID.talonSRX();
    protected final Solenoid piston = HardwareDevices.intakePistonID.solenoid();
    protected final DigitalInput sensor = new DigitalInput(9);
    // protected final LaserCan sensor = HardwareDevices.intakeSensorID.laserCan();

    public IntakeIOTalon() {
        motor.setInverted(true);
        motor.configContinuousCurrentLimit(6);
    }

    // private final Joystick a = new Joystick(0);
    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.motor.updateFrom(motor);
        // inputs.proximity.mut_replace(sensor.getMeasurement().distance_mm, Millimeters);
        // inputs.proximity.mut_replace(10*(1-a.getRawAxis(2)), Inches);
        inputs.sensorDetect = !sensor.get();
    }

    @Override
    public void setMotorVoltage(Measure<VoltageUnit> volts) {
        motor.set(ControlMode.PercentOutput, volts.in(Volts) / 12);
    }

    @Override
    public void setOpen(boolean open) {
        piston.set(open ^ IntakeConstants.pistonInverted);
    }
}
