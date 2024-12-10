package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.auto.AutoCommons.AutoPaths;
import frc.robot.auto.AutoCommons.StartPosition;
import frc.robot.auto.AutoSelector.AutoQuestion;
import frc.robot.auto.AutoSelector.AutoQuestion.Settings;
import frc.robot.auto.AutoSelector.AutoRoutine;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.puncher.Puncher;

public class ScoreStacking extends AutoRoutine {
    private static final AutoQuestion<StartPosition> startPosition = new AutoQuestion<>(
        "Start Position",
        () -> {
            var outer = StartPosition.Outer.toEntry();
            return Settings.from(outer, outer);
        }
    );
    // private static final AutoQuestion<Bucket> bucket1 = new AutoQuestion<>(
    //     "Bucket 1",
    //     () -> {
    //         var bucket2 = Bucket.Bucket2.toEntry();
    //         var bucket3 = Bucket.Bucket3.toEntry();
    //         var bucket4 = Bucket.Bucket4.toEntry();
    //         return Settings.from(bucket4, bucket2, bucket3, bucket4);
    //     }
    // );
    // private static final AutoQuestion<Bucket> bucket2 = new AutoQuestion<>(
    //     "Bucket 2",
    //     () -> {
    //         var bucket2 = Bucket.Bucket2.toEntry();
    //         var bucket3 = Bucket.Bucket3.toEntry();
    //         var bucket4 = Bucket.Bucket4.toEntry();
    //         return Settings.from(bucket4, bucket2, bucket3, bucket4);
    //     }
    // );

    private final Drive drive;
    private final Arm arm;
    private final Intake intake;
    private final Puncher puncher;

    public ScoreStacking(RobotContainer robot) {
        super(
            "Score Stacking",
            List.of(
                startPosition
                // bucket1,
                // bucket2
            )
        );
        this.drive = robot.drive;
        this.arm = robot.arm;
        this.intake = robot.intake;
        this.puncher = robot.puncher;
    }

    @Override
    public Command generateCommand() {
        var startPosition = ScoreStacking.startPosition.getResponse();
        // var bucket1 = ScoreStacking.bucket1.getResponse();
        // var bucket2 = ScoreStacking.bucket2.getResponse();

        var commands = new ArrayList<Command>(8);

        commands.add(AutoCommons.stack(intake, arm));

        var scoreToBucket1 = AutoPaths.loadChoreoTrajectory("Stacking0 To Bucket2");
        var bucket1ToScore = AutoPaths.loadChoreoTrajectory("Bucket2 To Stacking1");
        commands.add(AutoCommons.scoreStagedBucketStacking(scoreToBucket1, bucket1ToScore, drive, intake, arm, puncher));

        var scoreToBucket2 = AutoPaths.loadChoreoTrajectory("Stacking1 To Bucket1");
        var bucket2ToScore = AutoPaths.loadChoreoTrajectory("Bucket1 To Stacking2");
        commands.add(AutoCommons.scoreStagedBucketStacking(scoreToBucket2, bucket2ToScore, drive, intake, arm, puncher));

        return AutoCommons
            .setOdometryFlipped(startPosition.startPose, drive)
            .andThen(commands.toArray(Command[]::new))
        ;
    }
}
