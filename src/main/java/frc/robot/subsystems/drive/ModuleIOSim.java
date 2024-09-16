package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class ModuleIOSim implements ModuleIO {
    // jKg constants unknown, stolen from Mechanical Advnatage
    private FlywheelSim driveSim = new FlywheelSim(DCMotor.getFalcon500(1), DriveConstants.driveWheelGearReduction, 0.025);
    private FlywheelSim turnSim  = new FlywheelSim(DCMotor.getFalcon500(1), DriveConstants.turnWheelGearReduction,  0.004);

    private double turnRelativePositionRad = 0.0;
    private double turnAbsolutePositionRad = Math.random() * 2.0 * Math.PI;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    private boolean zeroEncodersFlag = false;

    public void updateInputs(ModuleIOInputs inputs) {
        driveSim.update(Constants.dtSeconds);
        turnSim.update(Constants.dtSeconds);
    
        double angleDiffRad = turnSim.getAngularVelocityRadPerSec() * Constants.dtSeconds;
        turnRelativePositionRad += angleDiffRad;
        turnAbsolutePositionRad += angleDiffRad;
        turnAbsolutePositionRad = MathUtil.angleModulus(turnAbsolutePositionRad);

        if (zeroEncodersFlag) {
          inputs.driveMotor.positionRad = 0.0;
          turnAbsolutePositionRad -= turnRelativePositionRad;
          turnRelativePositionRad = 0.0;
          zeroEncodersFlag = false;
        }
        
        inputs.driveMotor.updateFrom(driveSim, turnAppliedVolts);
    
        inputs.turnMotor.updateFrom(turnSim, turnAppliedVolts);
        inputs.turnMotor.positionRad = turnRelativePositionRad;
      }
    
      public void setDriveVoltage(double volts) {
        driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        driveSim.setInputVoltage(driveAppliedVolts);
      }
    
      public void setTurnVoltage(double volts) {
        turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        turnSim.setInputVoltage(turnAppliedVolts);
      }

      public void zeroEncoders() {
        zeroEncodersFlag = true;        
      }
}
