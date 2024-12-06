package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.auto.AutoCommons.AutoPaths;
import frc.robot.auto.AutoCommons.Bucket;
import frc.robot.auto.AutoCommons.StartPosition;
import frc.robot.auto.AutoSelector.AutoQuestion;
import frc.robot.auto.AutoSelector.AutoQuestion.Settings;
import frc.robot.auto.AutoSelector.AutoRoutine;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.puncher.Puncher;

public class ScoreHigh extends AutoRoutine {
    private static final AutoQuestion<StartPosition> startPosition = new AutoQuestion<>(
        "Start Position",
        () -> {
            var inner = StartPosition.Inner.toEntry();
            var outer = StartPosition.Outer.toEntry();
            return Settings.from(inner, inner, outer);
        }
    );
    private static final AutoQuestion<Bucket> bucket1 = new AutoQuestion<>(
        "Bucket 1",
        () -> {
            var bucket2 = Bucket.Bucket2.toEntry();
            var bucket3 = Bucket.Bucket3.toEntry();
            var bucket4 = Bucket.Bucket4.toEntry();
            return Settings.from(bucket4, bucket2, bucket3, bucket4);
        }
    );
    private static final AutoQuestion<Bucket> bucket2 = new AutoQuestion<>(
        "Bucket 2",
        () -> {
            var bucket2 = Bucket.Bucket2.toEntry();
            var bucket3 = Bucket.Bucket3.toEntry();
            var bucket4 = Bucket.Bucket4.toEntry();
            return Settings.from(bucket4, bucket2, bucket3, bucket4);
        }
    );

    private final Drive drive;
    private final Arm arm;
    private final Intake intake;
    private final Puncher puncher;

    public ScoreHigh(RobotContainer robot) {
        super(
            "Score High",
            List.of(
                startPosition,
                bucket1,
                bucket2
            )
        );
        this.drive = robot.drive;
        this.arm = robot.arm;
        this.intake = robot.intake;
        this.puncher = robot.puncher;
    }

    @Override
    public Command generateCommand() {
        var startPosition = ScoreHigh.startPosition.getResponse();
        var bucket1 = ScoreHigh.bucket1.getResponse();
        var bucket2 = ScoreHigh.bucket2.getResponse();

        var commands = new ArrayList<Command>(8);

        var startToScore = AutoPaths.loadChoreoTrajectory(switch (startPosition) {
            case Inner -> "Inner To Stacking";
            default -> "Outer To Stacking";
        });
        commands.add(AutoCommons.scorePreloadHigh(startToScore, drive, intake, puncher));

        var scoreToBucket1 = AutoPaths.loadChoreoTrajectory(switch (bucket1) {
            case Bucket2 -> "Stacking To Bucket2";
            case Bucket3 -> "Stacking To Bucket3";
            default -> "Stacking To Bucket4";
        });
        var bucket1ToScore = AutoPaths.loadChoreoTrajectory(switch (bucket1) {
            case Bucket2 -> "Bucket2 To Stacking";
            case Bucket3 -> "Bucket3 To Stacking";
            default -> "Bucket4 To Stacking";
        });
        commands.add(AutoCommons.scoreStagedBucketHigh(scoreToBucket1, bucket1ToScore, drive, intake, arm, puncher));

        var scoreToBucket2 = AutoPaths.loadChoreoTrajectory(switch (bucket2) {
            case Bucket2 -> "Stacking To Bucket2";
            case Bucket3 -> "Stacking To Bucket3";
            default -> "Stacking To Bucket4";
        });
        var bucket2ToScore = AutoPaths.loadChoreoTrajectory(switch (bucket2) {
            case Bucket2 -> "Bucket2 To Stacking";
            case Bucket3 -> "Bucket3 To Stacking";
            default -> "Bucket4 To Stacking";
        });
        commands.add(AutoCommons.scoreStagedBucketHigh(scoreToBucket2, bucket2ToScore, drive, intake, arm, puncher));

        return AutoCommons
            .setOdometryFlipped(startPosition.startPose, drive)
            .andThen(commands.toArray(Command[]::new))
        ;
    }
}
