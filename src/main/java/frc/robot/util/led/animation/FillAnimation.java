package frc.robot.util.led.animation;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.led.strips.LEDStrip;

public class FillAnimation extends LEDAnimation {
    private final LEDStrip[] strips;
    private final Supplier<Color> color;
    
    public FillAnimation(int priority, Color color, LEDStrip... strips) {
        this(priority, () -> color, strips);
    }

    public FillAnimation(int priority, Supplier<Color> color, LEDStrip... strips) {
        super(priority);
        this.strips = strips;
        this.color = color;
    }

    @Override
    public void execute() {
        for(LEDStrip ledStrip : strips) {
            ledStrip.foreach((int i) -> {
                ledStrip.setLED(i, color.get());
            });
        }
    }
}
