package frc.robot.subsystems.drive.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.drive.Drive;

public class DriveToPose extends Command {
    private final Drive drive;
    private final Pose2d target;
    private Pose2d initialPose;
    private State currentState;
    private final TrapezoidProfile linearProfile = new TrapezoidProfile(
        new Constraints(
            MetersPerSecond.of(2),
            MetersPerSecondPerSecond.of(4)
        )
    );

    public DriveToPose(Drive drive, Pose2d target) {
        this.drive = drive;
        this.target = target;
        setName("Drive to " + this.target);
        addRequirements(this.drive.translationSubsystem, this.drive.rotationalSubsystem);
    }

    @Override
    public void initialize() {
        initialPose = drive.getPose();
    }

    @Override
    public void execute() {
        currentState = linearProfile.calculate(
            RobotConstants.rioUpdatePeriodSecs,
            currentState,
            new State(
                target.minus(initialPose).getTranslation().getNorm(),
                0
            )
        );
        
    }
}
