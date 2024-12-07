package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
    private static final AutoQuestion<Boolean> stackingGoal = new AutoQuestion<>(
        "Target Goal",
        () -> {
            var stacking = Map.entry("Stacking", true);
            var source = Map.entry("Source", false);
            return Settings.from(source, source, stacking);
        }
    );
    private static final AutoQuestion<Optional<Bucket>> bucket1 = new AutoQuestion<>(
        "Bucket 1",
        () -> {
            var bucket2 = Map.entry("2", Optional.of(Bucket.Bucket2));
            var bucket3 = Map.entry("3", Optional.of(Bucket.Bucket3));
            var bucket4 = Map.entry("4", Optional.of(Bucket.Bucket4));
            var noBucket = Map.entry("N/A", Optional.<Bucket>empty());
            return Settings.from(bucket4, bucket2, bucket3, bucket4, noBucket);
        }
    );
    private static final AutoQuestion<Optional<Bucket>> bucket2 = new AutoQuestion<>(
        "Bucket 2",
        () -> {
            var bucket2 = Map.entry("2", Optional.of(Bucket.Bucket2));
            var bucket3 = Map.entry("3", Optional.of(Bucket.Bucket3));
            var bucket4 = Map.entry("4", Optional.of(Bucket.Bucket4));
            var noBucket = Map.entry("N/A", Optional.<Bucket>empty());
            return Settings.from(bucket3, bucket2, bucket3, bucket4, noBucket);
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
                stackingGoal,
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
        var stackingGoal = ScoreHigh.stackingGoal.getResponse();
        var bucket1 = ScoreHigh.bucket1.getResponse();
        var bucket2 = ScoreHigh.bucket2.getResponse();

        var commands = new ArrayList<Command>(8);
        var goalName = (stackingGoal ? "Stacking" : "Source");

        var startToScore = AutoPaths.loadChoreoTrajectory(switch (startPosition) {
            case Outer -> "Outer To " + goalName;
            default -> "Inner To " + goalName;
        });
        commands.add(Commands.deadline(
            AutoCommons.scorePreloadHigh(startToScore, drive, intake, puncher),
            arm.puncher()
        ));
        if (bucket1.isPresent()) {
            var scoreToBucket1 = AutoPaths.loadChoreoTrajectory(switch (bucket1.get()) {
                case Bucket2 -> goalName + " To Bucket2";
                case Bucket3 -> goalName + " To Bucket3";
                default -> goalName + " To Bucket4";
            });
            var bucket1ToScore = AutoPaths.loadChoreoTrajectory(switch (bucket1.get()) {
                case Bucket2 -> "Bucket2 To " + goalName;
                case Bucket3 -> "Bucket3 To " + goalName;
                default -> "Bucket4 To " + goalName;
            });
            commands.add(AutoCommons.scoreStagedBucketHigh(scoreToBucket1, bucket1ToScore, drive, intake, arm, puncher));
        }

        if (bucket2.isPresent()) {
            var scoreToBucket2 = AutoPaths.loadChoreoTrajectory(switch (bucket2.get()) {
                case Bucket2 -> goalName + " To Bucket2";
                case Bucket3 -> goalName + " To Bucket3";
                default -> goalName + " To Bucket4";
            });
            var bucket2ToScore = AutoPaths.loadChoreoTrajectory(switch (bucket2.get()) {
                case Bucket2 -> "Bucket2 To " + goalName;
                case Bucket3 -> "Bucket3 To " + goalName;
                default -> "Bucket4 To " + goalName;
            });
            commands.add(AutoCommons.scoreStagedBucketHigh(scoreToBucket2, bucket2ToScore, drive, intake, arm, puncher));
        }

        return AutoCommons
            .setOdometryFlipped(startPosition.startPose, drive)
            .andThen(commands.toArray(Command[]::new))
        ;
    }
}
