package frc.robot.util.led.strips;

import java.util.Arrays;
import java.util.function.IntConsumer;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.led.strips.software.ConcatenatedStrip;
import frc.robot.util.led.strips.software.ParallelStrip;
import frc.robot.util.led.strips.software.ReversedStrip;
import frc.robot.util.led.strips.software.SubStrip;

public interface LEDStrip {
    public int getLength();
    public static int getLength(LEDStrip[] strips) {
        return Arrays.stream(strips).mapToInt((s) -> s.getLength()).sum();
    }

    public void setLED(int ledIndex, Color color);

    public default void foreach(IntConsumer function) {
        for (int i = 0; i < getLength(); i++) {
            function.accept(i);
        }
    }
    public default void clear() {
        foreach((int i) -> setLED(i, Color.kBlack));
    }

    public default LEDStrip concat(LEDStrip... strips) {
        return new ConcatenatedStrip(this).concat(strips);
    }
    public default LEDStrip substrip(int startIndex) {
        return new SubStrip(startIndex, this);
    }
    public default LEDStrip substrip(int startIndex, int endIndex) {
        return new SubStrip(startIndex, endIndex, this);
    }
    public default LEDStrip reverse() {
        return new ReversedStrip(this);
    }
    public default LEDStrip parallel(LEDStrip... strips) {
        return new ParallelStrip(this).parallel(strips);
    }
}
