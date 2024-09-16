package frc.robot.util.led.strips.software;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.led.strips.LEDStrip;

public class ConcatenatedStrip implements LEDStrip {
    private final LEDStrip[] strips;
    private final int length;

    public ConcatenatedStrip(LEDStrip... strips) {
        this.strips = strips;
        int accumLength = 0;
        for(LEDStrip strip : strips) {
            accumLength += strip.getLength();
        }
        this.length = accumLength;
    }

    @Override
    public LEDStrip concat(LEDStrip... strips) {
        LEDStrip[] newStrips = new LEDStrip[strips.length + this.strips.length];
        for(int i = 0; i < newStrips.length; i++) {
            newStrips[i] = (i < this.strips.length ? newStrips[i] = this.strips[i] : strips[i - this.strips.length]);
        }
        return new ConcatenatedStrip(newStrips);
    }

    @Override
    public int getLength() {
        return length;
    }

    @Override
    public void setLED(int ledIndex, Color color) {
        int accumLength = 0;
        for(LEDStrip strip : strips) {
            int stripLength = strip.getLength();
            accumLength += stripLength;
            if(accumLength >= ledIndex) {
                accumLength -= stripLength;
                ledIndex -= accumLength;
                strip.setLED(ledIndex, color);
                break;
            }
        }
    }
}
