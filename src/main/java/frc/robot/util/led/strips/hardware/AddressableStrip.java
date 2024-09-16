package frc.robot.util.led.strips.hardware;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class AddressableStrip implements HardwareStrip {
    private final AddressableLED strip;
    private final AddressableLEDBuffer buffer;

    public AddressableStrip(int PWMPort, int length) {
        this.strip = new AddressableLED(PWMPort);
        this.buffer = new AddressableLEDBuffer(length);
        this.strip.setLength(this.buffer.getLength());
        this.strip.setData(this.buffer);
        this.strip.start();
    }

    @Override
    public int getLength() {
        return buffer.getLength();
    }

    @Override
    public void setLED(int ledIndex, Color color) {
        var color8bit = new Color8Bit(color);
        buffer.setRGB(ledIndex, color8bit.red, color8bit.green, color8bit.blue);
    }

    @Override
    public void refresh() {
        strip.setData(buffer);
    }
}
