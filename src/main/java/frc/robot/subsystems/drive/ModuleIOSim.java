package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.drive.DriveConstants.ModuleConstants;

public class ModuleIOSim extends ModuleIOFalcon550 {
    // jKg constants unknown, stolen from Mechanical Advnatage
    private final FlywheelSim driveSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getFalcon500(1), 0.025, 1/DriveConstants.driveWheelGearReduction),
        DCMotor.getFalcon500(1)
    );
    private final FlywheelSim turnSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getNeo550(1), 0.004, DriveConstants.turnWheelGearReduction),
        DCMotor.getFalcon500(1)
    );

    public ModuleIOSim(ModuleConstants moduleConstants) {
        super(moduleConstants);
    }

    private final MutAngle turnRelativePosition = Radians.mutable(0);
    private final MutAngle turnAbsolutePosition = Radians.mutable(Math.random() * 2.0 * Math.PI);
    private final MutVoltage turnAppliedVolts = Volts.mutable(0);

    // private boolean zeroEncodersFlag = false;

    public void updateInputs(ModuleIOInputs inputs) {
        var driveSimState = driveMotor.getSimState();
        if (DriverStation.isDisabled()) {
            turnAppliedVolts.mut_setBaseUnitMagnitude(0);
        }
        driveSim.setInputVoltage(driveSimState.getMotorVoltage());
        turnSim.setInputVoltage(turnAppliedVolts.in(Volts));
        
        driveSim.update(RobotConstants.rioUpdatePeriodSecs);
        turnSim.update(RobotConstants.rioUpdatePeriodSecs);

        double angleDiffRad = turnSim.getAngularVelocityRadPerSec() * RobotConstants.rioUpdatePeriodSecs;
        turnRelativePosition.mut_acc(angleDiffRad);
        turnAbsolutePosition.mut_acc(angleDiffRad);
        turnAbsolutePosition.mut_setMagnitude(MathUtil.angleModulus(turnAbsolutePosition.in(Radians)));

        // if (zeroEncodersFlag) {
        //     inputs.driveMotor.encoder.position.mut_setBaseUnitMagnitude(0.0);
        //     turnAbsolutePosition.mut_minus(turnRelativePosition);
        //     turnRelativePosition.mut_setMagnitude(0);
        //     zeroEncodersFlag = false;
        // }
        
        driveSimState.setSupplyVoltage(12 - driveSimState.getSupplyCurrent() * 0.002);

        super.updateInputs(inputs);

        inputs.turnMotor.updateFrom(turnSim, turnAppliedVolts);
        inputs.turnMotor.encoder.position.mut_replace(turnRelativePosition);
    }
    
    public void setTurnVoltage(Voltage volts) {
        turnAppliedVolts.mut_replace(MathUtil.clamp(volts.in(Volts), -12, 12), Volts);
    }

    // public void zeroEncoders() {
    //     zeroEncodersFlag = true;        
    // }
}
