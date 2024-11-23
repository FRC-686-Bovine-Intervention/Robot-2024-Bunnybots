package frc.robot.subsystems.puncher;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.constants.CANDevices;

public class PuncherIOReal implements PuncherIO {
    protected final Solenoid leftSolenoid = new Solenoid(PneumaticsModuleType.REVPH, CANDevices.leftSolenoidChannel);
    protected final Solenoid rightSolenoid = new Solenoid(PneumaticsModuleType.REVPH, CANDevices.rightSolenoidChannel);

    public PuncherIOReal() {
        this.retract();
    }
    
    @Override
    public void extend() {
        leftSolenoid.set(true);
        rightSolenoid.set(true);
    }

    @Override
    public void retract() {
        leftSolenoid.set(true);
        rightSolenoid.set(true);
    }

    @Override
    public void toggle() {
        leftSolenoid.toggle();
        rightSolenoid.toggle();
    }
}
