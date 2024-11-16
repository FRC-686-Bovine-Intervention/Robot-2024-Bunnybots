package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import frc.robot.constants.CANDevices;
import frc.util.LoggedTunableMeasure;

public class ArmIOFalcon implements ArmIO {
    protected final TalonFX motor = new TalonFX(CANDevices.armMotorID);
    protected final CANcoder encoder = new CANcoder(CANDevices.armEncoderID);

    private final LoggedTunableMeasure<AngleUnit> kP = new LoggedTunableMeasure<>("Pivot/PID/kP", Rotations.of(0));
    private final LoggedTunableMeasure<AngleUnit> kI = new LoggedTunableMeasure<>("Pivot/PID/kI", Rotations.of(0));
    private final LoggedTunableMeasure<AngleUnit> kD = new LoggedTunableMeasure<>("Pivot/PID/kD", Rotations.of(0));
    private final LoggedTunableMeasure<AngleUnit> kV = new LoggedTunableMeasure<>("Pivot/PID/Profile/kV", Radians.of(0));
    private final LoggedTunableMeasure<AngleUnit> kA = new LoggedTunableMeasure<>("Pivot/PID/Profile/kA", Radians.of(0));
    private final LoggedTunableMeasure<AngleUnit> kJ = new LoggedTunableMeasure<>("Pivot/PID/Profile/kJ", Radians.of(0));

    private final LoggedTunableMeasure<AngleUnit> ffkS = new LoggedTunableMeasure<>("Pivot/FF/kS", Rotations.of(0));
    private final LoggedTunableMeasure<AngleUnit> ffkG = new LoggedTunableMeasure<>("Pivot/FF/kG", Rotations.of(0));
    private final LoggedTunableMeasure<AngleUnit> ffkV = new LoggedTunableMeasure<>("Pivot/FF/kV", Rotations.of(0));
    private final LoggedTunableMeasure<AngleUnit> ffkA = new LoggedTunableMeasure<>("Pivot/FF/kA", Rotations.of(0));

    public ArmIOFalcon() {
        var motorConfig = new TalonFXConfiguration();
        motor.getConfigurator().refresh(motorConfig);
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.Feedback.RotorToSensorRatio = ArmConstants.motorToEncoderRatio.ratio();
        motorConfig.Feedback.SensorToMechanismRatio = ArmConstants.encoderToMechanismRatio.ratio();
        motorConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        motorConfig.Feedback.FeedbackRotorOffset = 0;
        motor.getConfigurator().apply(motorConfig);

        var encoderConfig = new CANcoderConfiguration();
        encoder.getConfigurator().refresh(encoderConfig);
        encoderConfig.MagnetSensor.MagnetOffset = 0;
        encoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
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
        if(
            kP.hasChanged(hashCode()) |
            kI.hasChanged(hashCode()) |
            kD.hasChanged(hashCode()) |
            kV.hasChanged(hashCode()) |
            kA.hasChanged(hashCode()) |
            kJ.hasChanged(hashCode()) |
            ffkV.hasChanged(hashCode()) |
            ffkA.hasChanged(hashCode()) |
            ffkG.hasChanged(hashCode()) |
            ffkS.hasChanged(hashCode())
        ) {
            var pidConfig = new Slot0Configs();
            var profileConfig = new MotionMagicConfigs();
            pidConfig.kP = kP.in(Radians);
            pidConfig.kI = kI.in(Radians);
            pidConfig.kD = kD.in(Radians);
            profileConfig.MotionMagicCruiseVelocity= kV.in(Rotations);
            profileConfig.MotionMagicAcceleration = kA.in(Rotations);
            profileConfig.MotionMagicJerk = kJ.in(Rotations);
            pidConfig.kV = ffkV.in(Radians);
            pidConfig.kA = ffkA.in(Radians);
            pidConfig.kG = ffkG.in(Radians);
            pidConfig.kS = ffkS.in(Radians);
            pidConfig.GravityType = GravityTypeValue.Arm_Cosine;

            motor.getConfigurator().apply(pidConfig);
            motor.getConfigurator().apply(profileConfig);
        }
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.motor.updateFrom(motor);
        inputs.encoder.updateFrom(encoder);

        updateTunables();
    }

    @Override
    public void setVoltage(Measure<VoltageUnit> volts) {
        motor.setVoltage(volts.in(Volts));
    }

    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
    @Override
    public void setPos(Measure<AngleUnit> pos) {
        motor.setControl(motionMagicRequest.withPosition((Angle) pos));
    }

    private final FeedbackConfigs motorFeedbackConfig = new FeedbackConfigs();
    @Override
    public void setOffset(Measure<AngleUnit> offset) {
        motor.getConfigurator().refresh(motorFeedbackConfig);
        var rotations = offset.in(Rotations);
        if (MathUtil.isNear(rotations, motorFeedbackConfig.FeedbackRotorOffset, 1e-3)) return;
        motorFeedbackConfig.FeedbackRotorOffset = rotations;
        motor.getConfigurator().apply(motorFeedbackConfig);
    }

    private static final ControlRequest COAST_OUT = new CoastOut();
    private static final ControlRequest NEUTRAL_OUT = new NeutralOut();
    @Override
    public void setCoast(boolean coast) {
        motor.setControl(coast ? COAST_OUT : NEUTRAL_OUT);
    }

    @Override
    public void stop() {
        motor.disable();
    }

    private final MagnetSensorConfigs encoderConfig = new MagnetSensorConfigs();
    @Override
    public void zero() {
        encoder.getConfigurator().refresh(encoderConfig);
        encoderConfig.MagnetOffset = encoder.getAbsolutePosition().getValue().in(Rotations);
        encoder.getConfigurator().apply(encoderConfig);
    }
}
