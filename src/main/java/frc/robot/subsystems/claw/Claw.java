package frc.robot.subsystems.claw;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
    private final ClawIO io;
    private final ClawIOInputsAutoLogged inputs = new ClawIOInputsAutoLogged();

    public Claw(ClawIO io) {
        System.out.println("[Init Claw] Instantiating Claw");
        this.io = io;
        System.out.println("[Init Claw] Claw IO: " + this.io.getClass().getSimpleName());
        SmartDashboard.putData("Subsystems/Claw", this);
    }
    
    public void periodic() {
        io.updateInputs(inputs);
    }
    
}
