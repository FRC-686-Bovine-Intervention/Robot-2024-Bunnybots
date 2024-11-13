package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.RobotConstants;

public class ArmIOSim extends ArmIOFalcon {
    private final SingleJointedArmSim armSim = new SingleJointedArmSim(
        DCMotor.getFalcon500(1),
        ArmConstants.motorToMechanismRatio.ratio(),
        ArmConstants.momentOfInertia.in(KilogramSquareMeters),
        ArmConstants.length.in(Meters),
        ArmConstants.minAngle.in(Radians),
        ArmConstants.minAngle.in(Radians),
        false,
        ArmConstants.startAngle.in(Radians)
    );
    
    @Override
    public void updateInputs(ArmIOInputs inputs) {
        var simState = motor.getSimState();
        var encoderSimState = encoder.getSimState();
        armSim.setInputVoltage(-simState.getMotorVoltage());
        armSim.update(RobotConstants.dtSeconds);

        var position = Rotations.convertFrom(armSim.getAngleRads(), Radians);
        var velocity = RotationsPerSecond.convertFrom(armSim.getVelocityRadPerSec(), RadiansPerSecond);

        encoderSimState.setRawPosition(position);
        encoderSimState.setVelocity(velocity);

        simState.setSupplyVoltage(12 - simState.getSupplyCurrent() * 0.002);

        super.updateInputs(inputs);
    }
}
