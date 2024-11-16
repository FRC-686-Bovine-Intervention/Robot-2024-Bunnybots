package frc.robot.auto;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.auto.AutoCommons.AutoPaths;
import frc.robot.auto.AutoCommons.Canister;
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

    private static final AutoQuestion<Canister> canister1 = generateNextCanister(1, List.of());
    private static final AutoQuestion<Canister> canister2 = generateNextCanister(2, List.of(canister1));
    private static final AutoQuestion<Canister> canister3 = generateNextCanister(3, List.of(canister1, canister2));
    private static final AutoQuestion<Canister> canister4 = generateNextCanister(4, List.of(canister1, canister2, canister3));

    private static AutoQuestion<Canister> generateNextCanister(int number, List<AutoQuestion<Canister>> prevQuestions) {
        return new AutoQuestion<>("Canister #" + number, 
        () -> {
            if (number > canisterCount.getResponse()) {
                return Settings.from(null, Map.of());
            }

            var prevResponses = prevQuestions.stream().map((canister) -> {
                return canister.getResponse();
            }).toList();
            Map<String, Canister> options = new LinkedHashMap<>();
            if (prevResponses.indexOf(Canister.Canister1) < 0) {
                var entry = Canister.Canister1.toEntry();
                options.put(entry.getKey(), entry.getValue());
            }
            if (prevResponses.indexOf(Canister.Canister2) < 0) {
                var entry = Canister.Canister2.toEntry();
                options.put(entry.getKey(), entry.getValue());
            }
            if (prevResponses.indexOf(Canister.Canister3) < 0) {
                var entry = Canister.Canister3.toEntry();
                options.put(entry.getKey(), entry.getValue());
            }
            if (prevResponses.indexOf(Canister.Canister4) < 0) {
                var entry = Canister.Canister4.toEntry();
                options.put(entry.getKey(), entry.getValue());
            }
            var defaultOption = options.entrySet().iterator().next();
            return Settings.from(defaultOption, options);
        });
    }

    public ScoreHigh(RobotContainer robot) {
        this(robot.drive);
    }

    private final Drive drive;

    public ScoreHigh(Drive drive) {
        super(
            "ScoreHigh",
            List.of(
                startPosition,
                canisterCount,
                canister1,
                canister2,
                canister3,
                canister4,
                canisterPickupPath,
                scoreEntryPath
            )
        );
        this.drive = drive;
    }

    @Override
    public Command generateCommand() {
        var startPosition = ScoreHigh.startPosition.getResponse();
        var canisterCount = ScoreHigh.canisterCount.getResponse();
        var canisterOrder = List.of(
            ScoreHigh.canister1.getResponse(),
            ScoreHigh.canister2.getResponse(),
            ScoreHigh.canister3.getResponse(),
            ScoreHigh.canister4.getResponse()
        );
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
