package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.wpilibj.Joystick;

public class IntakeIOSim implements IntakeIO {
    private final Joystick a = new Joystick(0);

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.proximity.mut_replace(10*(1-a.getRawAxis(2)), Inches);
    }
}
