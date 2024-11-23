package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.drive.DriveConstants.ModuleConstants;

public class ModuleIOSim extends ModuleIOFalcon550 {
    // jKg constants unknown, stolen from Mechanical Advnatage
    private final FlywheelSim driveSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getFalcon500(1), 0.0025/DriveConstants.driveWheelGearReduction/DriveConstants.driveWheelGearReduction, 1),
        DCMotor.getFalcon500(1)
    );
    private final FlywheelSim turnSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getNeo550(1), 0.004, DriveConstants.turnWheelGearReduction),
        DCMotor.getFalcon500(1)
    );

    public ModuleIOSim(ModuleConstants moduleConstants) {
        super(moduleConstants);
    }

    private final MutAngle driveRelativePosition = Radians.mutable(0);
    // private final MutAngle turnRelativePosition = Radians.mutable(0);
    private final MutAngle turnAbsolutePosition = Radians.mutable(0);
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

        var angleDiff = turnSim.getAngularVelocity().times(RobotConstants.rioUpdatePeriod);
        // turnRelativePosition.mut_acc(angleDiff);
        turnAbsolutePosition.mut_acc(angleDiff);
        turnAbsolutePosition.mut_setMagnitude(MathUtil.angleModulus(turnAbsolutePosition.in(Radians)));

        // if (zeroEncodersFlag) {
        //     inputs.driveMotor.encoder.position.mut_setBaseUnitMagnitude(0.0);
        //     turnAbsolutePosition.mut_minus(turnRelativePosition);
        //     turnRelativePosition.mut_setMagnitude(0);
        //     zeroEncodersFlag = false;
        // }
        var driveAngularDiff = driveSim.getAngularVelocity().times(RobotConstants.rioUpdatePeriod);
        driveRelativePosition.mut_acc(driveAngularDiff);
        driveSimState.setRawRotorPosition(driveRelativePosition);
        driveSimState.setRotorVelocity(driveSim.getAngularVelocity());
        driveSimState.setSupplyVoltage(12 - driveSimState.getSupplyCurrent() * 0.002);

        super.updateInputs(inputs);

        inputs.turnMotor.updateFrom(turnSim, turnAppliedVolts);
        inputs.turnMotor.encoder.position.mut_replace(turnAbsolutePosition);
        inputs.turnMotor.encoder.velocity.mut_replace(turnSim.getAngularVelocity());
    }
    
    @Override
    public void setTurnVolts(double volts) {
        turnAppliedVolts.mut_replace(MathUtil.clamp(volts, -12, 12), Volts);
    }
    @Override
    public void setTurnAngle(Measure<AngleUnit> angle) {
        setTurnVolts(
            turnPID.calculate(
                turnAbsolutePosition.in(Rotations),
                angle.in(Rotations)
            )
        );
    }

    // public void zeroEncoders() {
    //     zeroEncodersFlag = true;        
    // }
}
