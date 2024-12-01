package frc.robot.subsystems.drive.commands;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.stream.IntStream;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.Module;
import frc.util.LoggedTunableMeasure;
import frc.util.LoggedTunableNumber;
import frc.util.MathExtraUtil;

public class WheelRadiusCalibration extends Command {
    private final Drive drive;
    private final MutAngle prevYaw = Radians.mutable(0);
    private final MutAngle totalYaw = Radians.mutable(0);
    private final int maxSamples;
    private final Timer sampleTimer = new Timer();
    private final Measure<TimeUnit> samplePeriod;
    private final ArrayList<Sample> samples;
    private final Timer totalTimer = new Timer();
    private final Measure<VoltageUnit> maxVoltage;
    private final Measure<VelocityUnit<VoltageUnit>> voltageRampRate;
    private Angle[] initialPositions = new Angle[0];

    public static final LoggedTunableMeasure<VelocityUnit<VoltageUnit>> VOLTAGE_RAMP_RATE = new LoggedTunableMeasure<>("Drive/Wheel Calibration/Voltage Ramp Rate", Volts.per(Second).of(0));
    public static final LoggedTunableMeasure<VoltageUnit> MAX_VOLTAGE = new LoggedTunableMeasure<>("Drive/Wheel Calibration/Max Voltage", Volts.of(4));
    public static final LoggedTunableMeasure<TimeUnit> SAMPLE_PERIOD = new LoggedTunableMeasure<>("Drive/Wheel Calibration/Sample Period", Seconds.of(3));
    public static final LoggedTunableNumber MAX_SAMPLES = new LoggedTunableNumber("Drive/Wheel Calibration/Max Samples", 5);

    public WheelRadiusCalibration(Drive drive, int maxSamples, Measure<TimeUnit> samplePeriod, Measure<VelocityUnit<VoltageUnit>> voltageRampRate, Measure<VoltageUnit> maxVoltage) {
        this.drive = drive;
        addRequirements(this.drive.translationSubsystem, this.drive.rotationalSubsystem);
        setName("Wheel Calibration");
        this.maxSamples = maxSamples;
        this.samplePeriod = samplePeriod;
        this.samples = new ArrayList<>(this.maxSamples);
        this.voltageRampRate = voltageRampRate;
        this.maxVoltage = maxVoltage;
    }

    @Override
    public void initialize() {
        samples.clear();
        sampleTimer.restart();
        totalTimer.restart();
        prevYaw.mut_replace(drive.getYaw());
        initialPositions = Arrays.stream(drive.modules).map(Module::getWheelAngularPosition).map(Angle::copy).toArray(Angle[]::new);
    }

    @Override
    public void execute() {
        var yaw = drive.getYaw();
        var yawDiff = yaw.minus(prevYaw).in(Radians);
        var wrappedDiff = MathUtil.angleModulus(yawDiff);
        totalYaw.mut_acc(wrappedDiff);

        if (sampleTimer.advanceIfElapsed(samplePeriod.in(Seconds))) {
            sample();
        }
        prevYaw.mut_replace(yaw);

        var volts = Math.min(voltageRampRate.in(Volts.per(Second)) * totalTimer.get(), maxVoltage.in(Volts));
        Arrays.stream(drive.modules).forEach((module) -> module.runVoltage(Volts.of(volts), MathExtraUtil.rotationFromVector(module.config.positiveRotVec)));
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            sample();
        }

        sampleTimer.stop();
        totalTimer.stop();
        drive.driveVelocity(0,0,0);
    }

    @Override
    public boolean isFinished() {
        return samples.size() >= maxSamples;
    }

    private Sample sample() {
        var sample = new Sample(
            IntStream.range(0, drive.modules.length)
                .mapToObj((i) -> drive.modules[i].getWheelAngularPosition().minus(initialPositions[i]))
                .toArray(Angle[]::new)
            ,
            totalYaw.copy()
        );
        sample.print();
        samples.add(sample);
        return sample;
    }

    private static record Sample(
        Angle[] wheelPositions,
        Angle gyroPosition
    ) {
        public Distance calculateRadius() {
            var averagePosition = Arrays.stream(wheelPositions).mapToDouble(Angle::baseUnitMagnitude).average().orElse(0);
            return DriveConstants.driveBaseRadius.times(gyroPosition.baseUnitMagnitude()).divide(averagePosition);
        }

        public void print() {
            System.out.println("New Sample:");
            System.out.println("    Wheel Positions: " + Arrays.toString(wheelPositions));
            System.out.println("    Gyro Position: " + gyroPosition);
            System.out.println("    Wheel Radius: " + calculateRadius());
        }
    }
}
