package frc.robot.subsystems.objectiveTracker;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.util.VirtualSubsystem;

public class ObjectiveTracker extends VirtualSubsystem {
    
    public static enum Objective {
        HighGoal,
        StackingGrid,
        Yard,
        ;

    }
    private Objective currentObjective = Objective.HighGoal;
    public final Trigger highGoal = new Trigger(() -> currentObjective == Objective.HighGoal);
    public final Trigger stackingGrid = new Trigger(() -> currentObjective == Objective.StackingGrid);
    public final Trigger yard = new Trigger(() -> currentObjective == Objective.Yard);

    public Runnable setRunnable(Objective objective) {
        return () -> {
            currentObjective = objective;
        };
    }
    public Command set(Objective objective) {
        return Commands.runOnce(setRunnable(objective));
    }

    @Override
    public void periodic() {
        
    }
}
