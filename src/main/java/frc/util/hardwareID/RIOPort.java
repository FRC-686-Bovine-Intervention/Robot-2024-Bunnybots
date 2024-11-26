package frc.util.hardwareID;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogOutput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.PWM;

public class RIOPort {
    public final int port;

    private RIOPort(int port) {
        this.port = port;
    }

    public static RIOPort port(int port) {
        return new RIOPort(port);
    }

    public DigitalInput digitalInput() {
        return new DigitalInput(port);
    }
    public DigitalOutput digitalOutput() {
        return new DigitalOutput(port);
    }
    public AnalogInput analogInput() {
        return new AnalogInput(port);
    }
    public AnalogOutput AnalogOutput() {
        return new AnalogOutput(port);
    }
    public PWM pwm() {
        return new PWM(port);
    }
    public AddressableLED addressableLED() {
        return new AddressableLED(port);
    }
}
