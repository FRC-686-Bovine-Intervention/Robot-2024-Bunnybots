package frc.util.led.animation;

import java.util.function.DoubleFunction;

import edu.wpi.first.wpilibj.util.Color;
import frc.util.led.strips.LEDStrip;

public class BarAnimation extends LEDAnimation {
    private final LEDStrip strip;
    private final DoubleFunction<Color> gradient;
    private double barPos;

    public void setPos(double barPos) {
        this.barPos = barPos;
    }

    public BarAnimation(LEDStrip strip, DoubleFunction<Color> gradient) {
        this.strip = strip;
        this.gradient = gradient;
    }

    @Override
    public void apply() {
        strip.foreach((i) -> {
            var pixelPos = (double) i/strip.getLength();
            if (pixelPos <= barPos) {
                strip.setLED(i, gradient.apply(pixelPos));
            }
        });
    }
}
