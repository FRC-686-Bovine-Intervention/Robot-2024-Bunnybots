package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.RobotConstants;

public class ArmIOSim extends ArmIOFalcon {
    private final SingleJointedArmSim armSim = new SingleJointedArmSim(
        DCMotor.getFalcon500(1),
        ArmConstants.motorToMechanismRatio.ratio(),
        ArmConstants.momentOfInertia.in(KilogramSquareMeters),
        ArmConstants.length.in(Meters),
        ArmConstants.minAngle.in(Radians),
        ArmConstants.maxAngle.in(Radians),
        false,
        ArmConstants.startAngle.in(Radians)
    );
    
    @Override
    public void updateInputs(ArmIOInputs inputs) {
        var motorSimState = motor.getSimState();
        var encoderSimState = encoder.getSimState();
        armSim.setInputVoltage(motorSimState.getMotorVoltage());
        armSim.update(RobotConstants.rioUpdatePeriodSecs);

        var position = Radians.of(armSim.getAngleRads());
        var velocity = RadiansPerSecond.of(armSim.getVelocityRadPerSec());

        encoderSimState.setRawPosition(position);
        encoderSimState.setVelocity(velocity);

        motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        super.updateInputs(inputs);
    }
}
