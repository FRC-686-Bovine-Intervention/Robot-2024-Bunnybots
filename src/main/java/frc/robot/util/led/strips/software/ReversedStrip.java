package frc.robot.util.led.strips.software;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.led.strips.LEDStrip;

public class ReversedStrip implements LEDStrip {
    private final LEDStrip strip;

    public ReversedStrip(LEDStrip strip) {
        this.strip = strip;
    }

    @Override
    public LEDStrip reverse() {
        return strip;
    }

    @Override
    public int getLength() {
        return strip.getLength();
    }

    @Override
    public void setLED(int ledIndex, Color color) {
        strip.setLED(getLength() - ledIndex - 1, color);
    }
}
