// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.drive.DriveConstants.ModuleConstants;
import frc.util.CurrentSpikeDetector;
import frc.util.LoggedTunableMeasure;
import frc.util.LoggedTunableNumber;

public class Module {
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final ModuleConstants config;

    private static final LoggedTunableMeasure<DistanceUnit> wheelRadius = new LoggedTunableMeasure<>("Drive/Module/WheelRadius", Meters.of(DriveConstants.wheelRadius.in(Meters)));
    private static final LoggedTunableNumber driveKp = new LoggedTunableNumber("Drive/Module/Drive/kP", 0.1);
    private static final LoggedTunableNumber driveKd = new LoggedTunableNumber("Drive/Module/Drive/kD", 0.0);
    private static final LoggedTunableNumber driveKs = new LoggedTunableNumber("Drive/Module/Drive/kS", 0.18507);
    private static final LoggedTunableNumber driveKv = new LoggedTunableNumber("Drive/Module/Drive/kV", 0.08005);
    private static final LoggedTunableNumber turnKp = new LoggedTunableNumber("Drive/Module/Turn/kP", 5.0);
    private static final LoggedTunableNumber turnKd = new LoggedTunableNumber("Drive/Module/Turn/kD", 0.0);
    
    private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
    private final PIDController driveFeedback = new PIDController(0.0, 0.0, 0.0);
    private final PIDController turnFeedback = new PIDController(0.0, 0.0, 0.0);
    
    private SwerveModuleState state = new SwerveModuleState();
    private SwerveModulePosition modulePosition = new SwerveModulePosition();
    private SwerveModulePosition prevModulePosition = new SwerveModulePosition();

    private static final LoggedTunableMeasure<CurrentUnit> currentSpikeThreshold = new LoggedTunableMeasure<>("Drive/Current Spike Threshold", Amps.of(0)); 
    private static final LoggedTunableMeasure<TimeUnit> currentSpikeTime = new LoggedTunableMeasure<>("Drive/Current Spike Time", Seconds.of(0));
    private final CurrentSpikeDetector driveCurrentSpikeDetector = new CurrentSpikeDetector(currentSpikeThreshold, currentSpikeTime);

    public Module(ModuleIO io, ModuleConstants config) {
        this.io = io;
        this.config = config;

        turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
    }

    /** Updates inputs and checks tunable numbers. */
    public void periodic() {
        prevModulePosition = getPosition();

        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Drive/Module " + config.name, inputs);

        // Update controllers if tunable numbers have changed
        if (driveKp.hasChanged(hashCode()) | driveKd.hasChanged(hashCode())) {
            driveFeedback.setPID(driveKp.get(), 0.0, driveKd.get());
        }
        if (turnKp.hasChanged(hashCode()) | turnKd.hasChanged(hashCode())) {
            turnFeedback.setPID(turnKp.get(), 0.0, turnKd.get());
        }
        if (driveKs.hasChanged(hashCode()) | driveKv.hasChanged(hashCode())) {
            driveFeedforward = new SimpleMotorFeedforward(driveKs.get(), driveKv.get());
        }

        var angle = Rotation2d.fromRadians(MathUtil.angleModulus(inputs.turnMotor.encoder.position.in(Radians)));
        state = new SwerveModuleState(inputs.driveMotor.encoder.velocity.in(RadiansPerSecond) * wheelRadius.in(Meters), angle);
        modulePosition = new SwerveModulePosition(inputs.driveMotor.encoder.position.in(Radians) * wheelRadius.in(Meters), angle);

        driveCurrentSpikeDetector.update(getDriveCurrent());
    }

    /**
     * Runs the module with the specified setpoint state. Must be called
     * periodically. Returns the
     * optimized state.
     */
    public void runSetpoint(SwerveModuleState setpoint) {
        // Optimize state based on current angle
        setpoint.optimize(getAngle());

        // Run turn controller
        io.setTurnVoltage(Volts.of(turnFeedback.calculate(getAngle().getRadians(), setpoint.angle.getRadians())));

        // Update velocity based on turn error
        setpoint.speedMetersPerSecond *= Math.cos(turnFeedback.getError());

        // Run drive controller
        double velocityRadPerSec = setpoint.speedMetersPerSecond / wheelRadius.in(Meters);
        io.setDriveVoltage(
            driveFeedforward.calculate(RadiansPerSecond.of(velocityRadPerSec))
            .plus(Volts.of(driveFeedback.calculate(inputs.driveMotor.encoder.velocity.in(RadiansPerSecond), velocityRadPerSec))
        ));
    }

    /**
     * Runs the module with the specified voltage while controlling to zero degrees.
     * Must be called periodically.
     */
    public void runVoltage(Voltage volts, Rotation2d moduleAngle) {
        io.setTurnVoltage(Volts.of(turnFeedback.calculate(getAngle().getRadians(), moduleAngle.getRadians())));
        io.setDriveVoltage(volts);
    }

    /** Disables all outputs to motors. */
    public void stop() {
        io.stop();
    }

    /** Sets whether brake mode is enabled. */
    public void setBrakeMode(Boolean enabled) {
        io.setDriveBrakeMode(enabled);
        io.setTurnBrakeMode(enabled);
    }

    /** Returns the current turn angle of the module. */
    public Rotation2d getAngle() {
        return modulePosition.angle;
    }

    /** Returns the current drive position of the module in radians. */
    public Angle getAngularPosition() {
        return inputs.driveMotor.encoder.position;
    }

    public Current getDriveCurrent() {
        return inputs.driveMotor.motor.current;
    }

    public boolean currentSpiking() {
        return driveCurrentSpikeDetector.hasSpike();
    }

    /** Returns the module position (turn angle and drive position). */
    public SwerveModulePosition getPosition() {
        return modulePosition;
    }

    /** Returns the module state (turn angle and drive velocity). */
    public SwerveModuleState getState() {
        return state;
    }

    /** Returns change in module position since last tick */
    public SwerveModulePosition getPositionDelta() {
        var currentModulePosition = getPosition();
        return new SwerveModulePosition(currentModulePosition.distanceMeters - prevModulePosition.distanceMeters,
                currentModulePosition.angle);
    }

    /** Returns the drive velocity in radians/sec. */
    public double getCharacterizationVelocity() {
        return inputs.driveMotor.encoder.velocity.in(RadiansPerSecond);
    }

    /** Returns the drive wheel radius. */
    public static double getWheelRadius() {
        return wheelRadius.in(Meters);
    }

    /** Zeros module encoders. */
    public void zeroEncoders() {
        io.zeroEncoders();
        // need to also reset prevModulePosition because drive is driven by deltas in
        // position
        prevModulePosition = getPosition();
    }
}
