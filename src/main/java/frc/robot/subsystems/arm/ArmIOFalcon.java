package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

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
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import frc.robot.constants.CANDevices;
import frc.util.LoggedTunableMeasure;
import frc.util.loggerUtil.LoggedTunableFF;
import frc.util.loggerUtil.LoggedTunablePID;

public class ArmIOFalcon implements ArmIO {
    protected final TalonFX motor = new TalonFX(CANDevices.armMotorID, CANDevices.canivoreBusName);
    protected final CANcoder encoder = new CANcoder(CANDevices.armEncoderID, CANDevices.canivoreBusName);
    
    private final LoggedTunableMeasure<AngularVelocityUnit> profilekV = new LoggedTunableMeasure<>("Arm/Falcon/PID/Profile/kV", RadiansPerSecond.of(0));
    private final LoggedTunableMeasure<AngularAccelerationUnit> profilekA = new LoggedTunableMeasure<>("Arm/Falcon/PID/Profile/kA", RadiansPerSecondPerSecond.of(0));
    // private final LoggedTunableMeasure<AngleUnit> kJ = new LoggedTunableMeasure<>("Pivot/PID/Profile/kJ", Radians.of(0));
    private final LoggedTunablePID pidConsts = new LoggedTunablePID(
        "Arm/Falcon/PID",
        0,
        0,
        0
    );
    private final LoggedTunableFF ffConsts = new LoggedTunableFF(
        "Arm/Falcon/FF",
        0,
        0,
        0,
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
        // motorConfig.SoftwareLimitSwitch
        //     .withForwardSoftLimitEnable(true)
        //     .withReverseSoftLimitEnable(true)
        // ;
        motor.getConfigurator().apply(motorConfig);

        var encoderConfig = new CANcoderConfiguration();
        encoder.getConfigurator().refresh(encoderConfig);
        encoderConfig.MagnetSensor
            .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf)
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
        if (LoggedTunableMeasure.hasChanged(hashCode(), profilekV, profilekV)) {
            var motionMagic = new MotionMagicConfigs();
            motor.getConfigurator().refresh(motionMagic);
            motionMagic
                .withMotionMagicCruiseVelocity(profilekV.in(RotationsPerSecond))
                .withMotionMagicAcceleration(profilekA.in(RotationsPerSecondPerSecond))
            ;
            motor.getConfigurator().apply(motionMagic);
        }
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.motor.updateFrom(motor);
        inputs.encoder.updateFrom(encoder);

        updateTunables();
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
