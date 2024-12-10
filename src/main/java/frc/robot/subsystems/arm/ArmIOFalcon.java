package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import frc.robot.constants.HardwareDevices;
import frc.util.loggerUtil.tunables.LoggedTunableAngularProfile;
import frc.util.loggerUtil.tunables.LoggedTunableFF;
import frc.util.loggerUtil.tunables.LoggedTunablePID;

public class ArmIOFalcon implements ArmIO {
    protected final TalonFX motor = HardwareDevices.armMotorID.talonFX();
    protected final CANcoder encoder = HardwareDevices.armEncoderID.cancoder();
    
    private final LoggedTunableAngularProfile profileConsts = new LoggedTunableAngularProfile(
        "Arm/Falcon/Profile",
        RadiansPerSecond.of(4),
        RadiansPerSecondPerSecond.of(8)
    );
    private final LoggedTunablePID pidConsts = new LoggedTunablePID(
        "Arm/Falcon/PID",
        15,
        0,
        0

    );
    private final LoggedTunableFF ffConsts = new LoggedTunableFF(
        "Arm/Falcon/FF",
        0,
        0,
        12,
        0
    );

    public ArmIOFalcon() {
        var motorConfig = new TalonFXConfiguration();

        motorConfig.MotorOutput
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)
        ;
        motorConfig.Feedback
            .withRemoteCANcoder(encoder)
            .withRotorToSensorRatio(ArmConstants.motorToEncoderRatio.ratio())
            .withSensorToMechanismRatio(ArmConstants.encoderToMechanismRatio.ratio())
            // .withFeedbackRotorOffset(null)
        ;
        motorConfig.SoftwareLimitSwitch
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(ArmConstants.maxAngle)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(ArmConstants.minAngle)
        ;
        ffConsts.update(motorConfig.Slot0);
        pidConsts.update(motorConfig.Slot0);
        profileConsts.update(motorConfig.MotionMagic);
        motor.getConfigurator().apply(motorConfig);

        var encoderConfig = new CANcoderConfiguration();
        encoder.getConfigurator().refresh(encoderConfig);
        encoderConfig.MagnetSensor
            .withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.5))
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
        ;
        encoder.getConfigurator().apply(encoderConfig);

        updateTunables();

        BaseStatusSignal.setUpdateFrequencyForAll(
            ArmConstants.MotorSignalFrequency,
            motor.getPosition(),
            motor.getVelocity(),
            motor.getClosedLoopError()
        );
    }

    private void updateTunables() {
        if (pidConsts.hasChanged(hashCode()) | ffConsts.hasChanged(hashCode())) {
            var slotConfig = new SlotConfigs();
            motor.getConfigurator().refresh(slotConfig);
            pidConsts.update(slotConfig);
            ffConsts.update(slotConfig);
            motor.getConfigurator().apply(slotConfig);
        }
        if (profileConsts.hasChanged(hashCode())) {
            var motionMagicConfigs = new MotionMagicConfigs();
            motor.getConfigurator().refresh(motionMagicConfigs);
            profileConsts.update(motionMagicConfigs);
            motor.getConfigurator().apply(motionMagicConfigs);
        }
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.motor.updateFrom(motor);
        inputs.encoder.updateFrom(encoder);

        updateTunables();

        Logger.recordOutput("Arm/Profile/Position", Rotations.of(motor.getClosedLoopReference().getValueAsDouble()));
        Logger.recordOutput("Arm/Profile/Velocity", RotationsPerSecond.of(motor.getClosedLoopReferenceSlope().getValueAsDouble()));
        Logger.recordOutput("Arm/PID/Error", Rotations.of(motor.getClosedLoopError().getValueAsDouble()));
        Logger.recordOutput("Arm/PID/P", Volts.of(motor.getClosedLoopProportionalOutput().getValueAsDouble()));
    }


    private final VoltageOut voltageOut = new VoltageOut(0);
    @Override
    public void setVoltage(Measure<VoltageUnit> volts) {
        motor.setControl(
            voltageOut.withOutput(volts.in(Volts))
        );
    }

    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
    @Override
    public void setAngle(Measure<AngleUnit> pos) {
        motor.setControl(motionMagicRequest.withPosition(pos.in(Rotations)));
    }

    private static final CoastOut COAST_OUT = new CoastOut();
    private static final NeutralOut NEUTRAL_OUT = new NeutralOut();
    @Override
    public void setCoast(boolean coast) {
        motor.setControl(coast ? COAST_OUT : NEUTRAL_OUT);
    }

    @Override
    public void stop() {
        motor.disable();
    }

    // private final MagnetSensorConfigs encoderConfig = new MagnetSensorConfigs();
    // @Override
    // public void zero() {
    //     encoder.getConfigurator().refresh(encoderConfig);
    //     encoderConfig.MagnetOffset = encoder.getAbsolutePosition().getValue().in(Rotations);
    //     encoder.getConfigurator().apply(encoderConfig);
    // }
}
