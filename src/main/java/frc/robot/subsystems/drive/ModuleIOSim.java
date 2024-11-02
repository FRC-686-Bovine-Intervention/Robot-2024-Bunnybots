package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class ModuleIOSim implements ModuleIO {
    // jKg constants unknown, stolen from Mechanical Advnatage
    private final FlywheelSim driveSim = new FlywheelSim(DCMotor.getFalcon500(1), DriveConstants.driveWheelGearReduction, 0.025);
    private final FlywheelSim turnSim  = new FlywheelSim(DCMotor.getFalcon500(1), DriveConstants.turnWheelGearReduction,  0.004);

    private final MutAngle turnRelativePosition = Radians.mutable(0);
    private final MutAngle turnAbsolutePosition = Radians.mutable(Math.random() * 2.0 * Math.PI);
    private final MutVoltage driveAppliedVolts = Volts.mutable(0);
    private final MutVoltage turnAppliedVolts = Volts.mutable(0);

    private boolean zeroEncodersFlag = false;

    public void updateInputs(ModuleIOInputs inputs) {
        driveSim.update(Constants.dtSeconds);
        turnSim.update(Constants.dtSeconds);

        double angleDiffRad = turnSim.getAngularVelocityRadPerSec() * Constants.dtSeconds;
        turnRelativePosition.mut_acc(angleDiffRad);
        turnAbsolutePosition.mut_acc(angleDiffRad);
        turnAbsolutePosition.mut_setMagnitude(MathUtil.angleModulus(turnAbsolutePosition.in(Radians)));

        if (zeroEncodersFlag) {
            inputs.driveMotor.encoder.position.mut_setBaseUnitMagnitude(0.0);
            turnAbsolutePosition.mut_minus(turnRelativePosition);
            turnRelativePosition.mut_setMagnitude(0);
            zeroEncodersFlag = false;
        }
        
        inputs.driveMotor.updateFrom(driveSim, turnAppliedVolts);

        inputs.turnMotor.updateFrom(turnSim, turnAppliedVolts);
        inputs.turnMotor.encoder.position.mut_replace(turnRelativePosition);
    }
    
    public void setDriveVoltage(Voltage volts) {
        driveAppliedVolts.mut_replace(MathUtil.clamp(volts.in(Volts), -12, 12), Volts);
        driveSim.setInputVoltage(driveAppliedVolts.in(Volts));
    }

    public void setTurnVoltage(Voltage volts) {
        turnAppliedVolts.mut_replace(MathUtil.clamp(volts.in(Volts), -12, 12), Volts);
        turnSim.setInputVoltage(turnAppliedVolts.in(Volts));
    }

    public void zeroEncoders() {
        zeroEncodersFlag = true;        
    }
}
