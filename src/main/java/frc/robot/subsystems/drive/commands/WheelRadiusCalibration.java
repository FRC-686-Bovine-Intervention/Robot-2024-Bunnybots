package frc.robot.subsystems.drive.commands;

import static edu.wpi.first.units.Units.Radians;

import java.util.ArrayList;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class WheelRadiusCalibration extends Command {
    private final Drive drive;
    private final int maxSamples;
    private final ArrayList<Object> samples;
    private final Timer timer = new Timer();
    private final MutAngle prevYaw = Radians.mutable(0);
    private final MutAngle totalYaw = Radians.mutable(0);

    public WheelRadiusCalibration(Drive drive, int maxSamples) {
        this.drive = drive;
        addRequirements(this.drive.translationSubsystem, this.drive.rotationalSubsystem);
        setName("Wheel Calibration");
        this.maxSamples = maxSamples;
        this.samples = new ArrayList<>(this.maxSamples);
    }

    @Override
    public void initialize() {
        samples.clear();
        timer.restart();
        prevYaw.mut_replace(drive.getYaw());
    }

    @Override
    public void execute() {
        var yaw = drive.getYaw();
        var yawDiff = yaw.minus(prevYaw).in(Radians);
        var wrappedDiff = MathUtil.angleModulus(yawDiff);
        totalYaw.mut_acc(wrappedDiff);

        if (timer.advanceIfElapsed(1)) {
            var sample = 5;
        }
        prevYaw.mut_replace(prevYaw);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        drive.driveVelocity(0,0,0);
    }

    @Override
    public boolean isFinished() {
        return samples.size() >= maxSamples;
    }
}
