//grippers go!
package frc.robot.subsystems.claw;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.LoggedTunableNumber;

public class Claw {
    // Initial variable setup
    private ClawIO clawIO;
    
    // Periodic Function - runs constantly
    public void periodic() {
        // Update inputs
        manipIO.updateInputs(manipIOInputs);
        Logger.processInputs("Manip", manipIOInputs);
    }
    
}
