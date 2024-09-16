package frc.robot.subsystems.drive.commands;

import java.util.Arrays;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class WheelCalibration extends Command {
    private final Drive.Translational drive;
    private final Timer timer = new Timer();

    private static final ChassisSpeeds driveSpeed = new ChassisSpeeds(1, 0, 0);

    private double accum;

    private double timeEnd;

    public WheelCalibration(Drive.Translational drive) {
        addRequirements(drive);
        this.drive = drive;
    }

    @Override
    public void initialize() {
        timer.restart();
        accum = 0;
        timeEnd = Double.MAX_VALUE;
    }

    @Override
    public void execute() {
        accum += getCurrentDistDelta();
        if(getDist() >= 3) {
            if(timeEnd > 50000000) {
                timeEnd = timer.get();
            }
            drive.stop();
        } else {
            drive.driveVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(driveSpeed, drive.drive.getRotation()));
        }
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        System.out.println("[Wheel Calibration]");
        System.out.println("Timer = " + timer.get());
        System.out.println("Dist = " + getDist());
    }

    private double getCurrentDistDelta() {
        return Arrays.stream(drive.drive.getModulePositionDeltas()).mapToDouble((a) -> Math.abs(a.distanceMeters)).average().getAsDouble();
    }

    private double getDist() {
        return accum;
    }

    @Override
    public boolean isFinished() {
        return timer.get() - timeEnd > 1;
    }
}
