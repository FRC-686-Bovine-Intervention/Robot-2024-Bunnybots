package frc.robot.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.SuppliedEdgeDetector;
import frc.robot.util.VirtualSubsystem;

public class AutoManager extends VirtualSubsystem {
    private final AutoSelector selector;

    private Command autonomousCommand;
    private final SuppliedEdgeDetector autoEnabled = new SuppliedEdgeDetector(DriverStation::isAutonomousEnabled);

    public AutoManager(AutoSelector selector) {
        this.selector = selector;
    }

    @Override
    public void periodic() {
        autoEnabled.update();
        if(autoEnabled.risingEdge()) {
            autonomousCommand = selector.getSelectedAutoCommand();
            if(autonomousCommand != null) {
                autonomousCommand.schedule();
            }
        }
        if(autoEnabled.fallingEdge() && autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }
}
