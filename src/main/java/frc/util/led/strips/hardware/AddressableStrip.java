package frc.util.led.strips.hardware;

import java.util.stream.IntStream;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.util.hardwareID.RIOPort;

public class AddressableStrip implements HardwareStrip {
    private final AddressableLED strip;
    private final AddressableLEDBuffer buffer;

    public AddressableStrip(RIOPort port, int length) {
        this.strip = port.addressableLED();
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
        Logger.recordOutput("LEDS/Strip", IntStream.range(0, buffer.getLength()).mapToObj((index) -> buffer.getLED(index).toHexString()).toArray(String[]::new));
    }
}
