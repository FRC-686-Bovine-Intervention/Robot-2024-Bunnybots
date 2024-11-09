package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Robot;
import frc.robot.constants.CANDevices;
import frc.robot.subsystems.drive.DriveConstants.ModuleConstants;
import frc.util.Alert;
import frc.util.Alert.AlertType;

public class ModuleIOFalcon550 implements ModuleIO {
    private final TalonFX  driveMotor;
    private final CANSparkMax turnMotor;
    private final AbsoluteEncoder turnAbsoluteEncoder;
    private final Angle initialTurnOffset;

    public ModuleIOFalcon550(ModuleConstants config) {
        driveMotor = new TalonFX(config.driveMotorID, CANDevices.driveCanBusName);
        turnMotor = new CANSparkMax(config.turnMotorID, MotorType.kBrushless);
        turnAbsoluteEncoder = turnMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        initialTurnOffset = config.cancoderOffset;

        /** Configure Drive Motors */
        var driveConfig = new TalonFXConfiguration();
        // change factory defaults here
        driveConfig.MotorOutput.Inverted = config.driveInverted;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.MotorOutput.DutyCycleNeutralDeadband = 0.0;
        driveConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.1875;
        driveConfig.CurrentLimits.SupplyCurrentLimit = 55;
        driveConfig.CurrentLimits.SupplyCurrentLowerLimit = 55;
        driveConfig.CurrentLimits.SupplyCurrentLowerTime = 0;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveConfig.CurrentLimits.StatorCurrentLimit = 55;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveMotor.getConfigurator().apply(driveConfig);

        /** Configure Turn Motors */
        turnMotor.setInverted(false);
        turnMotor.setIdleMode(IdleMode.kBrake);
        turnMotor.setSmartCurrentLimit(40);
        turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
        turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);

        setFramePeriods(driveMotor, true);

        zeroEncoders();

        tempWarning = new Alert(config.name + " Module has exceeded 70C", AlertType.WARNING);
        tempAlert = new Alert(config.name + " Module has exceeded 100C", AlertType.ERROR);
    }

    private final Alert tempWarning;
    private final Alert tempAlert;

    public void updateInputs(ModuleIOInputs inputs) {
        inputs.driveMotor.updateFrom(driveMotor);
        inputs.driveMotor.encoder.position.mut_divide(DriveConstants.driveWheelGearReduction);
        inputs.driveMotor.encoder.velocity.mut_divide(DriveConstants.driveWheelGearReduction);

        inputs.turnMotor.updateFrom(turnMotor);
        inputs.turnMotor.encoder.position.mut_replace(MathUtil.angleModulus(Units.rotationsToRadians(turnAbsoluteEncoder.getPosition())) - initialTurnOffset.in(Radians), Radians);

        tempWarning.set(inputs.driveMotor.motor.temperature.in(Celsius) > 70);
        tempAlert.set(driveMotor.getFault_DeviceTemp().getValue());
    }

    public void zeroEncoders() {
        driveMotor.setPosition(0.0);
        // turnRelativeEncoder.setPosition(turnAbsoluteEncoder.getPosition());
    }

    public void setDriveVoltage(Voltage volts) {
        driveMotor.setVoltage(volts.in(Volts));
    }

    public void setTurnVoltage(Voltage volts) {
        turnMotor.setVoltage(volts.in(Volts));
    }

    private static void setFramePeriods(TalonFX talon, boolean needMotorSensor) {
        // reduce rates of most status frames

        // TODO: revisit figuring out what getters to slow down
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255, 1000);
        // if (!needMotorSensor) {
        //    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255, 1000);
        // }
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255, 1000);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255, 1000);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 255, 1000);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 255, 1000);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255, 1000);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 255, 1000);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255, 1000);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 255, 1000);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255, 1000);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255, 1000);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255, 1000);

        talon.getPosition().setUpdateFrequency(Robot.defaultPeriodSecs);
    }

    @Override
    public void setDriveBrakeMode(Boolean enable) {
        driveMotor.setControl(enable == null ? new NeutralOut() : (enable.booleanValue() ? new StaticBrake() : new CoastOut()));
    }

    @Override
    public void setTurnBrakeMode(Boolean enable) {
        if(enable == null) return;
        turnMotor.setIdleMode(enable.booleanValue() ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void stop() {
        var driveRequest = driveMotor.getAppliedControl();
        if(driveRequest instanceof VoltageOut) {
            driveMotor.setControl(new NeutralOut());
        }
        setTurnVoltage(Volts.zero());
    }
}
