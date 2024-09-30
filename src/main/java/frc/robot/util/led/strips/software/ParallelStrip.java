package frc.robot.util.led.strips.software;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.led.strips.LEDStrip;

public class ParallelStrip implements LEDStrip {
    private final LEDStrip[] strips;
    private final int length;

    public ParallelStrip(LEDStrip... strips) {
        this.strips = strips;
        int maxLength = 0;
        for(LEDStrip strip : strips) {
            maxLength = Math.max(maxLength, strip.getLength());
        }
        this.length = maxLength;
    }

    @Override
    public LEDStrip parallel(LEDStrip... strips) {
        LEDStrip[] newStrips = new LEDStrip[strips.length + this.strips.length];
        for(int i = 0; i < newStrips.length; i++) {
            newStrips[i] = (i < this.strips.length ? newStrips[i] = this.strips[i] : strips[i - this.strips.length]);
        }
        return new ParallelStrip(newStrips);
    }

    @Override
    public int getLength() {
        return length;
    }

    @Override
    public void setLED(int ledIndex, Color color) {
        for(LEDStrip strip : strips) {
            strip.setLED(ledIndex, color);
        }
    }
}
