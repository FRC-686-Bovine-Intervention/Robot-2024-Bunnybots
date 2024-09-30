package frc.robot.util.led.animation;

import java.util.Set;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.util.led.strips.LEDStrip;

public abstract class LEDAnimation extends Command {
    public LEDAnimation(int priority) {
        this.priority = priority;
    }

    public int priority;
    public final int getPriority() {return priority;}
    public final void setPriority(int priority) {this.priority = priority;}

    public final Timer animationTimer = new Timer();

    @Override
    public void schedule() {
        LEDManager.getInstance().schedule(this);
    }
    @Override
    public void cancel() {
        LEDManager.getInstance().cancel(this);
    }
    @Override
    public boolean isScheduled() {
        return LEDManager.getInstance().isScheduled(this);
    }
    @Override
    public final boolean hasRequirement(Subsystem requirement) {
        return false;
    }
    @Override
    public final boolean runsWhenDisabled() {
        return true;
    }
    @Override
    public final Set<Subsystem> getRequirements() {
        return Set.of();
    }
    @Override
    public ParallelRaceGroup withTimeout(double seconds) {
        return asProxy().withTimeout(seconds);
    }

    public static class StripCounterAnimation extends LEDAnimation {
        private final LEDStrip[] strips;

        public StripCounterAnimation(int priority, LEDStrip... strips) {
            super(priority);
            this.strips = strips;
        }

        @Override
        public void execute() {
            for(LEDStrip ledStrip : strips) {
                for(int i = 0; i < ledStrip.getLength(); i++) {
                    int ind = i + 1;
                    if(ind % 50 == 0) {
                        ledStrip.setLED(i, Color.kGreen);
                    } else if(ind % 10 == 0) {
                        ledStrip.setLED(i, Color.kRed);
                    } else if(ind % 5 == 0) {
                        ledStrip.setLED(i, Color.kBlue);
                    } else {
                        ledStrip.setLED(i, Color.kBlack);
                    }
                }
            }
        }

    }
}
