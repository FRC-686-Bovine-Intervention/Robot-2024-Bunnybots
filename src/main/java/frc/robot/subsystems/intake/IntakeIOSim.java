package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Joystick;

public class IntakeIOSim implements IntakeIO {
    private final Joystick a = new Joystick(0);

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.sensorDetect = a.getRawAxis(2) > 0.5;
    }
}
