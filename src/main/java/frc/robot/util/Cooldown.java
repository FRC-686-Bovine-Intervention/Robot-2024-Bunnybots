package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;

public class Cooldown {
    private final Timer timer = new Timer();
    private final double defaultCooldown;
    private double cooldown = 0;

    public Cooldown() {
        this(0);
    }

    public Cooldown(double defaultCooldown) {
        this.defaultCooldown = defaultCooldown;
    }

    public boolean hasExpired() {
        var ret = timer.hasElapsed(cooldown);
        if (ret) {
            timer.stop();
        }
        return ret;
    }

    public double secondsRemaining() {
        return cooldown - timer.get();
    }
    
    public void reset() {
        reset(defaultCooldown);
    }
    
    public void reset(double cooldown) {
        this.cooldown = cooldown;
        timer.restart();
    }
}
