package frc.robot.util.led.animation;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.led.strips.LEDStrip;

public class BarAnimation extends LEDAnimation {
    public static class BarData {
        public final Color color;
        public final double percentage;
        public BarData(Color color, double percentage) {
            this.color = color;
            this.percentage = percentage;
        }
    }
    private final LEDStrip[] strips;
    private final Supplier<List<BarData>> barSupplier;
    private final Optional<Color> backgroundColor;
    
    public BarAnimation(int priority, Supplier<List<BarData>> barSupplier, Optional<Color> backgroundColor, LEDStrip... strips) {
        super(priority);
        this.strips = strips;
        this.barSupplier = barSupplier;
        this.backgroundColor = backgroundColor;
    }

    @Override
    public void execute() {
        var bars = barSupplier.get().stream().sorted((BarData a, BarData b) -> (int)Math.signum(a.percentage - b.percentage)).toList();
        for(LEDStrip ledStrip : strips) {
            int barIndex = 0;
            for(int i = 0; i < ledStrip.getLength(); i++) {
                Color color = backgroundColor.get();
                double ledPercentage = i / Math.max(ledStrip.getLength() - 1.0, 1);
                while(barIndex < bars.size()) {
                    var tryBar = bars.get(barIndex);
                    if(tryBar.percentage > ledPercentage) {
                        color = tryBar.color;
                        break;
                    }
                    barIndex++;
                }
                if(color != null) {
                    ledStrip.setLED(i, color);
                }
            }
        }
    }
}
