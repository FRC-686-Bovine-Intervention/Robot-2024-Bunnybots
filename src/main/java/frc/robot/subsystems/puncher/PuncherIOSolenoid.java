package frc.robot.subsystems.puncher;

import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.constants.HardwareDevices;

public class PuncherIOSolenoid implements PuncherIO {
    protected final Solenoid puncher = HardwareDevices.puncherPistonID.solenoid();

    public PuncherIOSolenoid() {

    }

    @Override
    public void updateInputs(PuncherIOInputs inputs) {
        
    }

    @Override
    public void setDeployed(boolean deployed) {
        puncher.set(deployed ^ PuncherConstants.pistonInverted);
    }
}
