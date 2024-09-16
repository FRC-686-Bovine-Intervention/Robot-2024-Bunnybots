package frc.robot.subsystems.leds;

import frc.robot.util.VirtualSubsystem;
import frc.robot.util.led.animation.LEDManager;

public class Leds extends VirtualSubsystem {
    private final LEDManager ledManager = LEDManager.getInstance();
    public Leds() {}

    @Override
    public void periodic() {
        ledManager.run();
    }
}
