package frc.robot.util.led.strips.hardware;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.util.led.strips.LEDStrip;

public class CANdleStrip implements HardwareStrip {
    private final CANdle candle;
    private final int length;
    private final Color8Bit[] ledBuffer;
    private final Color8Bit[] diffBuffer;
    private static final int LEDsPerFrame = 60;
    private int bufferPos = 0;

    public CANdleStrip(CANdle candle, int offboardStripLength) {
        this.candle = candle;
        this.length = Math.max(offboardStripLength, 0) + 8;
        this.ledBuffer = new Color8Bit[this.length];
        this.diffBuffer = new Color8Bit[this.length];
    }

    @Override
    public int getLength() {
        return length;
    }

    @Override
    public void setLED(int ledIndex, Color color) {
        var curColor = ledBuffer[ledIndex];
        var color8bit = new Color8Bit(color);
        if(!color.equals(curColor)) {
            diffBuffer[ledIndex] = color8bit;
        } else {
            diffBuffer[ledIndex] = null;
        }
    }

    @Override
    public void refresh() {
        // CANdles can only update a certain amount of LEDs per frame, so this will only update that many and continue next frame
        for(int i = 0, updatesThisFrame = 0; updatesThisFrame < LEDsPerFrame && i < length; i++, bufferPos = ++bufferPos % length) {
            var color = diffBuffer[bufferPos];
            if(color == null) continue;
            updatesThisFrame++;
            candle.setLEDs(color.red, color.green, color.blue, 0, bufferPos, 1);
            ledBuffer[bufferPos] = color;
            diffBuffer[bufferPos] = null;
        }
    }

    public LEDStrip getOnboardLEDs() {
        return this.substrip(0, 8);
    }

    public LEDStrip getOffboardLEDs() {
        return this.substrip(8);
    }
}
