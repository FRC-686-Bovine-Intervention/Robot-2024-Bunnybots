package frc.robot.util.led.animation;

import java.util.function.DoubleFunction;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.led.functions.TilingFunction;
import frc.robot.util.led.strips.LEDStrip;

public class FlashingAnimation extends LEDAnimation {
    private final LEDStrip[] strips;
    private final DoubleFunction<Color> gradient;
    private final TilingFunction tilingFunction;

    private double              period = 1;
    public double               getPeriod()                 {return period;}
    public FlashingAnimation    setPeriod(double period)    {this.period = period; return this;}

    public FlashingAnimation(int priority, DoubleFunction<Color> gradient, TilingFunction tilingFunction, LEDStrip... strips) {
        super(priority);
        this.strips = strips;
        this.gradient = gradient;
        this.tilingFunction = tilingFunction;
    }

    @Override
    public void execute() {
        for(LEDStrip ledStrip : strips) {
            ledStrip.foreach((int i) -> {
                ledStrip.setLED(i, gradient.apply(tilingFunction.tile(animationTimer.get()/period)));
            });
        }
    }
}
