package frc.robot.subsystems.claw;

public class ClawIO {
    public static class ManipulatorIOInputs {
        public double manipAppliedVolts = 0.0;
        public double manipCurrentAmps = 0.0;
    }

    public default void updateInputs(ManipulatorIOInputs inputs) {}

    public default void setVoltage(double volts) {}
}
