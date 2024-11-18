package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.CANDevices;
import frc.robot.subsystems.drive.DriveConstants.ModuleConstants;
import frc.util.LoggedTunableNumber;
import frc.util.TalonFXTempAlerts;
import frc.util.loggerUtil.LoggedTunableFF;
import frc.util.loggerUtil.LoggedTunablePID;

public class ModuleIOFalcon550 implements ModuleIO {
    protected final TalonFX driveMotor;
    protected final SparkMax turnMotor;
    protected final AbsoluteEncoder turnAbsoluteEncoder;

    private final TalonFXTempAlerts tempAlerts;

    private final VoltageOut driveVolts = 
        new VoltageOut(0)
        .withOverrideBrakeDurNeutral(true)
    ;
    private final MotionMagicVelocityVoltage driveVelocity = 
        new MotionMagicVelocityVoltage(0)
        .withAcceleration(0)
        .withOverrideBrakeDurNeutral(true)
    ;
    private final StaticBrake driveBrake = new StaticBrake();
    private final NeutralOut driveNeutral = new NeutralOut();


    private static final LoggedTunableNumber driveProfilekA = new LoggedTunableNumber("Drive/Module/Drive/PID/Profile/kA", 50);
    private static final LoggedTunablePID drivePIDConsts = new LoggedTunablePID(
        "Drive/Module/Drive/PID",
        5*2*Math.PI,
        0*2*Math.PI,
        0*2*Math.PI
    );
    private static final LoggedTunableFF driveFFConsts = new LoggedTunableFF(
        "Drive/Module/Drive/FF",
        0.18507*2*Math.PI,
        0*2*Math.PI,
        0.08005*2*Math.PI,
        0*2*Math.PI
    );
    private static final LoggedTunablePID turnPIDConsts = new LoggedTunablePID(
        "Drive/Module/Turn/PID",
        5*2*Math.PI,
        0*2*Math.PI,
        0*2*Math.PI
    );
    private final PIDController turnPID = new PIDController(0, 0, 0);

    public ModuleIOFalcon550(ModuleConstants config) {
        driveMotor = new TalonFX(config.driveMotorID, CANDevices.canivoreBusName);
        turnMotor = new SparkMax(config.turnMotorID, MotorType.kBrushless);
        turnAbsoluteEncoder = turnMotor.getAbsoluteEncoder();

        var driveConfig = new TalonFXConfiguration();
        driveConfig.MotorOutput
            .withInverted(config.driveInverted)
            .withNeutralMode(NeutralModeValue.Coast)
            .withDutyCycleNeutralDeadband(0)
        ;
        driveConfig.OpenLoopRamps
            .withVoltageOpenLoopRampPeriod(Seconds.of(0.1875))
        ;
        driveConfig.CurrentLimits
            .withSupplyCurrentLimit(Amps.of(55))
            .withSupplyCurrentLowerLimit(Amps.of(55))
            .withSupplyCurrentLowerTime(Seconds.of(0))
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(Amps.of(55))
            .withStatorCurrentLimitEnable(true)
        ;
        driveConfig.MotionMagic
            .withMotionMagicAcceleration(RadiansPerSecondPerSecond.of(0))
            // .withMotionMagicJerk(RadiansPerSecondPerSecond.per(Second).of(1))
        ;

        driveMotor.getConfigurator().apply(driveConfig);

        var turnConfig = new SparkMaxConfig();
        turnConfig.idleMode(IdleMode.kCoast)
            .inverted(false)
            .smartCurrentLimit(40)
            .absoluteEncoder
                .inverted(true)
            // .signals
            //     .absoluteEncoderPositionPeriodMs((int) RobotConstants.rioUpdatePeriod.in(Milliseconds))
        ;
        // turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
        // turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);

        turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turnPID.enableContinuousInput(
            0,
            1
        );
        SmartDashboard.putData("Drive/" + config.name, turnPID);

        BaseStatusSignal.setUpdateFrequencyForAll(
            DriveConstants.odometryLoopFrequency,
            driveMotor.getPosition(),
            driveMotor.getVelocity()
        );

        // zeroEncoders();

        tempAlerts = new TalonFXTempAlerts(driveMotor, config.name + " Module");
    }

    public void updateInputs(ModuleIOInputs inputs) {
        if (drivePIDConsts.hasChanged(hashCode()) | driveFFConsts.hasChanged(hashCode())) {
            var slotConfig = new SlotConfigs();
            driveMotor.getConfigurator().refresh(slotConfig);
            drivePIDConsts.update(slotConfig);
            driveFFConsts.update(slotConfig);
            driveMotor.getConfigurator().apply(slotConfig);
        }
        if (turnPIDConsts.hasChanged(hashCode())) {
            turnPIDConsts.update(turnPID);
        }
        if (driveProfilekA.hasChanged(hashCode())) {
            var motionMagic = new MotionMagicConfigs();
            driveMotor.getConfigurator().refresh(motionMagic);
            motionMagic.withMotionMagicAcceleration(RadiansPerSecondPerSecond.of(driveProfilekA.get()));
            driveMotor.getConfigurator().apply(motionMagic);
        }
        
        inputs.driveMotor.updateFrom(driveMotor);
        // inputs.driveMotor.encoder.position.mut_divide(DriveConstants.driveWheelGearReduction);
        // inputs.driveMotor.encoder.velocity.mut_divide(DriveConstants.driveWheelGearReduction);

        inputs.turnMotor.updateFrom(turnMotor);
        // inputs.turnMotor.encoder.position.mut_replace(MathUtil.angleModulus(Units.rotationsToRadians(turnAbsoluteEncoder.getPosition())) - initialTurnOffset.in(Radians), Radians);

        tempAlerts.update();
    }

    // public void zeroEncoders() {
    //     driveMotor.setPosition(0.0);
    //     // turnRelativeEncoder.setPosition(turnAbsoluteEncoder.getPosition());
    // }

    public void setDriveVoltage(Voltage volts) {
        driveMotor.setControl(driveVolts.withOutput(volts));
    }
    public void setDriveVelocity(AngularVelocity velocity) {
        driveMotor.setControl(driveVelocity.withVelocity(velocity));
    }

    public void setTurnVoltage(Voltage volts) {
        turnMotor.setVoltage(volts.in(Volts));
    }
    public void setTurnAngle(Angle angle) {
        turnMotor.setVoltage(
            turnPID.calculate(
                turnAbsoluteEncoder.getPosition(),
                angle.in(Rotations)
            )
        );
    }
    
    @Override
    public void setDriveBrakeMode(boolean enable) {
        driveMotor.setControl(enable ? driveBrake : driveNeutral);
    }

    @Override
    public void setTurnBrakeMode(boolean enable) {
        //TODO Reimplement module turn brake mode
        // turnMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void stop() {
        driveMotor.stopMotor();
        setTurnVoltage(Volts.zero());
    }
}
