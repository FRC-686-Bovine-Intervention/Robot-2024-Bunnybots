package frc.util.led.animation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import frc.util.led.functions.Gradient;
import frc.util.led.functions.InterpolationFunction;
import frc.util.led.functions.WaveFunction;
import frc.util.led.strips.LEDStrip;

public class AllianceColorAnimation {
    private final LEDStrip strip;
    private final Gradient unknownGradient = new Gradient(InterpolationFunction.linear, Color.kBlue, Color.kRed);
    private final Gradient blueGradient = new Gradient(InterpolationFunction.linear, Color.kBlue, Color.kBlack);
    private final Gradient redGradient = new Gradient(InterpolationFunction.linear, Color.kRed, Color.kBlack);

    public AllianceColorAnimation(LEDStrip strip) {
        this.strip = strip;
    }

    public void apply() {
        strip.apply((pos) -> {
            var alliance = DriverStation.getAlliance();
            Gradient gradient;
            if (alliance.isEmpty()) {
                gradient = unknownGradient;
            } else {
                if (alliance.get() == Alliance.Blue) {
                    gradient = blueGradient;
                } else {
                    gradient = redGradient;
                }
            }
            return gradient.apply(
                WaveFunction.Sinusoidal.applyAsDouble(
                    pos * 4 - Timer.getFPGATimestamp()
                )
            );
        });
    }
}
