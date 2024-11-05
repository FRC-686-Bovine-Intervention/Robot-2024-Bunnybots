package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.auto.AutoCommons.AutoPaths;
import frc.robot.auto.AutoCommons.PathType;
import frc.robot.auto.AutoCommons.StartPosition;
import frc.robot.auto.AutoSelector.AutoQuestion;
import frc.robot.auto.AutoSelector.AutoQuestion.Settings;
import frc.robot.auto.AutoSelector.AutoRoutine;
import frc.robot.subsystems.drive.Drive;

public class ScoreHigh extends AutoRoutine {
    private static final AutoQuestion<StartPosition> startPosition = new AutoQuestion<>(
        "Start Position",
        () -> {
            var close = StartPosition.Close.toEntry();
            var middle = StartPosition.Middle.toEntry();
            var far = StartPosition.Far.toEntry();

            return Settings.from(close, close, middle, far);
        }
    );

    private static final AutoQuestion<Integer> canisterCount = new AutoQuestion<>(
        "Canister Count",
        () -> {
            var c0 = Map.entry("0", 0);
            var c1 = Map.entry("1", 1);
            var c2 = Map.entry("2", 2);
            var c3 = Map.entry("3", 3);
            var c4 = Map.entry("4", 4);

            return Settings.from(c4, c0, c1, c2, c3, c4);
        }
    );

    private static final ArrayList<AutoQuestion<Integer>> canisterOrder = new ArrayList<>();

    private static final Supplier<ArrayList<AutoQuestion<Integer>>> canisterOrderSupplier = () -> {
        var c1 = Map.entry("1", 1);
        var c2 = Map.entry("2", 2);
        var c3 = Map.entry("3", 3);
        var c4 = Map.entry("4", 4);

        List<Map.Entry<String, Integer>> initialOptions = List.of(c1, c2, c3, c4);

        for (int i = 0; i < canisterCount.getResponse(); i++) {
            List<Integer> previousResponses = new ArrayList<>();
            for (int j = 0; j < i; j++) {
                AutoQuestion<Integer> previousQuestion = canisterOrder.get(j);
                if (previousQuestion != null) {
                    Integer response = previousQuestion.getResponse();
                    if (response != null) {
                        previousResponses.add(response);
                    }
                }
            }

            List<Map.Entry<String, Integer>> options = new ArrayList<>(initialOptions);
            options.removeIf(entry -> previousResponses.contains(entry.getValue()));

            int count = i + 1;
            int defaultOption = options.get(0).getValue();
            
            @SuppressWarnings("unchecked")
            var newQuestion = new AutoQuestion<>(
                "Canister #" + count,
                () -> Settings.from(
                    Map.entry(String.valueOf(defaultOption), defaultOption),
                    (Map.Entry<String, Integer>[]) options.toArray(new Map.Entry[0])
                )
            );

            if (i >= canisterOrder.size()) canisterOrder.add(newQuestion);

            if (!canisterOrder.get(i).equals(newQuestion)) canisterOrder.set(i, newQuestion);
        }

        if (canisterOrder.size() > canisterCount.getResponse()) {
            canisterOrder.subList(canisterCount.getResponse(), canisterOrder.size()).clear();
        }
        
        return canisterOrder;
    };

    private static final AutoQuestion<PathType> canisterPickupPath = new AutoQuestion<>(
        "Canister Pickup Path",
        () -> {
            var close = PathType.Close.toEntry();
            var far = PathType.Far.toEntry();

            return Settings.from(close, close, far);
        }
    );

    private static final AutoQuestion<PathType> scoreEntryPath = new AutoQuestion<>(
        "Score Entry Path",
        () -> {
            var close = PathType.Close.toEntry();
            var far = PathType.Far.toEntry();

            return Settings.from(close, close, far);
        }
    );

    private static final Supplier<List<AutoQuestion<?>>> questions = () -> {
        List<AutoQuestion<?>> questionList = new ArrayList<>();
        questionList.add(startPosition);    
        questionList.add(canisterCount);
        questionList.add(canisterPickupPath);
        questionList.add(scoreEntryPath);
        questionList.addAll(canisterOrderSupplier.get());
    
        return questionList;
    };

    public ScoreHigh(RobotContainer robot) {
        this(robot.drive);
    }

    private final Drive drive;

    public ScoreHigh(Drive drive) {
        super("ScoreHigh", questions);
        this.drive = drive;
    }

    @Override
    public Command generateCommand() {
        var startPosition = ScoreHigh.startPosition.getResponse();
        var canisterCount = ScoreHigh.canisterCount.getResponse();
        var canisterOrder = ScoreHigh.canisterOrder.stream().map((canister) -> {
            return canister.getResponse();
        }).toList();
        var canisterPickupPath = ScoreHigh.canisterPickupPath.getResponse();
        var scoreEntryPath = ScoreHigh.canisterPickupPath.getResponse();

        if (canisterCount == 0) return Commands.none();

        var commands = new ArrayList<Command>();

        String startToCanisterTemplateString = "%s Start - Canister #%s %s Path";
        String canisterToScoreTemplateString = "Canister #%s - Score %s Path";
        String scoreToCanisterTemplateString = "Score - Canister #%s %s Path";


        commands.add(
            AutoCommons.followPathFlipped(
                AutoPaths.loadChoreoTrajectory(
                    String.format(
                        startToCanisterTemplateString,
                        startPosition,
                        canisterOrder.get(0),
                        canisterPickupPath
                    )
                ),
                drive
            )
        );

        commands.add(
            AutoCommons.followPathFlipped(
                AutoPaths.loadChoreoTrajectory(
                    String.format(
                        canisterToScoreTemplateString,
                        canisterOrder.get(0),
                        scoreEntryPath
                    )
                ),
                drive
            )
        );

        for (int i = 1; i < canisterOrder.size(); i++) {
            commands.add(
                AutoCommons.followPathFlipped(
                    AutoPaths.loadChoreoTrajectory(
                        String.format(
                            scoreToCanisterTemplateString,
                            canisterOrder.get(i),
                            canisterPickupPath
                        )
                    ),
                    drive
                )
            );

            commands.add(
                AutoCommons.followPathFlipped(
                    AutoPaths.loadChoreoTrajectory(
                        String.format(
                            canisterToScoreTemplateString,
                            canisterOrder.get(i),
                            scoreEntryPath
                        )
                    ),
                    drive
                )
            );
        }

        return AutoCommons
                .setOdometryFlipped(startPosition.startPose, drive)
                .andThen(commands.toArray(Command[]::new));
    }
}
