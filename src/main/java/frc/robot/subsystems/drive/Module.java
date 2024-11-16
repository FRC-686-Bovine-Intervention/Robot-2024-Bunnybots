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

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.drive.DriveConstants.ModuleConstants;
import frc.util.CurrentSpikeDetector;
import frc.util.LoggedTunableMeasure;

public class Module {
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final ModuleConstants config;

    private static final LoggedTunableMeasure<DistanceUnit> wheelRadius = new LoggedTunableMeasure<>("Drive/Module/WheelRadius", Meters.of(DriveConstants.wheelRadius.in(Meters)));
    
    private SwerveModuleState state = new SwerveModuleState();
    private SwerveModulePosition modulePosition = new SwerveModulePosition();
    private SwerveModulePosition prevModulePosition = new SwerveModulePosition();

    private static final LoggedTunableMeasure<CurrentUnit> currentSpikeThreshold = new LoggedTunableMeasure<>("Drive/Current Spike Threshold", Amps.of(0)); 
    private static final LoggedTunableMeasure<TimeUnit> currentSpikeTime = new LoggedTunableMeasure<>("Drive/Current Spike Time", Seconds.of(0));
    private final CurrentSpikeDetector driveCurrentSpikeDetector = new CurrentSpikeDetector(currentSpikeThreshold, currentSpikeTime);

    public Module(ModuleIO io, ModuleConstants config) {
        this.io = io;
        this.config = config;
    }

    /** Updates inputs and checks tunable numbers. */
    public void periodic() {
        prevModulePosition = getPosition();

        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Drive/Module " + config.name, inputs);

        var angle = Rotation2d.fromRadians(MathUtil.angleModulus(inputs.turnMotor.encoder.position.in(Radians)));
        state = new SwerveModuleState(inputs.driveMotor.encoder.velocity.in(RadiansPerSecond) * wheelRadius.in(Meters), angle);
        modulePosition = new SwerveModulePosition(inputs.driveMotor.encoder.position.in(Radians) * wheelRadius.in(Meters), angle);

        driveCurrentSpikeDetector.update(getDriveCurrent());
    }

    /**
     * Runs the module with the specified setpoint state. Must be called
     * periodically.
     */
    public void runSetpoint(SwerveModuleState setpoint) {
        setpoint.optimize(getAngle());

        var turnSetpoint = setpoint.angle.getMeasure().minus(config.encoderOffset);
        io.setTurnAngle(turnSetpoint);

        setpoint.speedMetersPerSecond *= Math.cos(turnSetpoint.minus(inputs.turnMotor.encoder.position).in(Radians));

        double velocityRadPerSec = setpoint.speedMetersPerSecond / wheelRadius.in(Meters);
        io.setDriveVelocity(RadiansPerSecond.of(velocityRadPerSec));
    }

    /**
     * Runs the module with the specified voltage while controlling to zero degrees.
     * Must be called periodically.
     */
    public void runVoltage(Voltage volts, Rotation2d moduleAngle) {
        io.setTurnAngle(moduleAngle.getMeasure().minus(config.encoderOffset));
        io.setDriveVoltage(volts);
    }

    /** Disables all outputs to motors. */
    public void stop() {
        io.stop();
    }

    /** Sets whether brake mode is enabled. */
    public void setBrakeMode(boolean enabled) {
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
        return new SwerveModulePosition(
            currentModulePosition.distanceMeters - prevModulePosition.distanceMeters,
            currentModulePosition.angle
        );
    }

    /** Returns the drive velocity in radians/sec. */
    public AngularVelocity getCharacterizationVelocity() {
        return inputs.driveMotor.encoder.velocity;
    }

    /** Returns the drive wheel radius. */
    public static double getWheelRadius() {
        return wheelRadius.in(Meters);
    }

    /** Zeros module encoders. */
    // public void zeroEncoders() {
    //     io.zeroEncoders();
    //     // need to also reset prevModulePosition because drive is driven by deltas in
    //     // position
    //     prevModulePosition = getPosition();
    // }
}
