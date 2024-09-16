package frc.robot.util.led.strips.hardware;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class SimulatedStrip implements HardwareStrip {
    private final AddressableLED strip;
    private final AddressableLEDSim sim;
    private final AddressableLEDBuffer buffer;
    private final byte[] data;

    public SimulatedStrip(int PWMPort, int length) {
        this.strip = new AddressableLED(PWMPort);
        this.sim = new AddressableLEDSim(strip);
        this.buffer = new AddressableLEDBuffer(length);
        this.strip.setLength(this.buffer.getLength());
        this.strip.setData(this.buffer);
        this.strip.start();
        this.data = new byte[this.buffer.getLength() * 4];
    }

    @Override
    public int getLength() {
        return buffer.getLength();
    }

    @Override
    public void setLED(int ledIndex, Color color) {
        buffer.setLED(ledIndex, color);
        var color8Bit = new Color8Bit(color);
        data[ledIndex * 4] = (byte) color8Bit.blue;
        data[(ledIndex * 4) + 1] = (byte) color8Bit.green;
        data[(ledIndex * 4) + 2] = (byte) color8Bit.red;
        data[(ledIndex * 4) + 3] = 0;
    }

    @Override
    public void refresh() {
        strip.setData(buffer);
        sim.setData(data);
    }
}
