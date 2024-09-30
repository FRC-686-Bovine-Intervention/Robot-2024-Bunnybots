package frc.robot.util.led.animation;

import java.util.ArrayList;
import java.util.Iterator;

import org.littletonrobotics.junction.Logger;

import frc.robot.util.led.strips.hardware.HardwareStrip;

public class LEDManager {
    private static LEDManager instance;
    public static LEDManager getInstance() {if(instance == null) instance = new LEDManager(); return instance;}

    private final ArrayList<LEDAnimation> scheduledAnimations = new ArrayList<>();
    private final ArrayList<HardwareStrip> registeredStrips = new ArrayList<>();

    public void run() {
        for(Iterator<LEDAnimation> iterator = scheduledAnimations
            .stream()
            .sorted((LEDAnimation a, LEDAnimation b) -> (a.getPriority() - b.getPriority()))
            .iterator();
            iterator.hasNext();
        ) {
            var anim = iterator.next();
            anim.execute();
            if(anim.isFinished()) {
                anim.end(false);
                Logger.recordOutput("Leds/" + anim.getName(), false);
                iterator.remove();
            }
        }
        
        registeredStrips.forEach(HardwareStrip::refresh);
    }

    public boolean isScheduled(LEDAnimation animation) {
        return scheduledAnimations.contains(animation);
    }

    public void schedule(LEDAnimation animation) {
        if(isScheduled(animation)) return;
        scheduledAnimations.add(animation);
        Logger.recordOutput("Leds/" + animation.getName(), true);
        animation.initialize();
        animation.animationTimer.start();
    }

    public void cancel(LEDAnimation animation) {
        if(!isScheduled(animation)) return;
        scheduledAnimations.remove(animation);
        animation.end(true);
        Logger.recordOutput("Leds/" + animation.getName(), false);
        animation.animationTimer.stop();
        animation.animationTimer.reset();
    }

    public boolean stopAll() {
        for (var animation : scheduledAnimations) {
            cancel(animation);
        }
        return scheduledAnimations.isEmpty();
    }

    public LEDManager register(HardwareStrip... hardwareStrips) {
        for(HardwareStrip hardwareStrip : hardwareStrips) {
            if(!registeredStrips.contains(hardwareStrip)) {
                registeredStrips.add(hardwareStrip);
            }
        }
        return this;
    }
}
