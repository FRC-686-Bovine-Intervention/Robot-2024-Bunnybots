package frc.robot.subsystems.claw;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.constants.CANDevices;

public class ClawIOTalonSRX implements ClawIO {
    protected final TalonSRX motor = new TalonSRX(CANDevices.clawMotorID);
    protected final Solenoid solenoid = new Solenoid(PneumaticsModuleType.REVPH, CANDevices.clawSolenoidChannel);

    public ClawIOTalonSRX() {}

    @Override
    public void updateInputs(ManipulatorIOInputs inputs) {
        inputs.motor.updateFrom(motor);
    }

    @Override
    public void setMotorVoltage(Measure<VoltageUnit> volts) {
        motor.set(ControlMode.PercentOutput, volts.in(Volts)/12);
    }

    @Override
    public void open() {
        solenoid.set(false);
    }

    @Override
    public void close() {
        solenoid.set(true);
    }
}
