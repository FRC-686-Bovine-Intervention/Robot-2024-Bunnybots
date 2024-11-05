package frc.robot.auto;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.util.SwitchableChooser;
import frc.util.VirtualSubsystem;

public class AutoSelector extends VirtualSubsystem {
    private final LoggedDashboardChooser<AutoRoutine> routineChooser;
    private final StringPublisher configPublisher;
    private final List<StringPublisher> questionPublishers;
    private final List<SwitchableChooser> responseChoosers;
    private final String key;
    
    private static final AutoRoutine defaultRoutine = new AutoRoutine("Do Nothing", () -> List.of()) {
        public Command generateCommand() {
            return Commands.idle();
        }
    };
    private final String questionPlaceHolder = "NA"; 

    private Command lastCommand;
    private AutoConfiguration lastConfiguration;

    public AutoSelector(String key) {
        this.key = key;
        routineChooser = new LoggedDashboardChooser<>(key + "/Routine");
        routineChooser.addDefaultOption(defaultRoutine.name, defaultRoutine);
        questionPublishers = new ArrayList<>();
        responseChoosers = new ArrayList<>();
        configPublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable(key).getStringTopic("Configuration").publish();
    }

    private void populateQuestions(AutoRoutine routine) {
        for(int i = questionPublishers.size(); i < routine.questions.get().size(); i++) {
            var publisher =
                NetworkTableInstance.getDefault()
                    .getStringTopic("/SmartDashboard/" + key + "/Question #" + Integer.toString(i + 1))
                    .publish()
            ;
            publisher.set(questionPlaceHolder);
            questionPublishers.add(publisher);
            responseChoosers.add(new SwitchableChooser(key + "/Question #" + Integer.toString(i + 1) + " Chooser"));
        }
    }

    public void addRoutine(AutoRoutine routine) {
        routineChooser.addOption(routine.name, routine);
    }

    public void addDefaultRoutine(AutoRoutine routine) {
        routineChooser.addDefaultOption(routine.name, routine);
    }

    @Override
    public void periodic() {
        var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        if(DriverStation.isEnabled()) return;
        var selectedRoutine = routineChooser.get();
        if(selectedRoutine == null) return;
        var config = new AutoConfiguration(alliance, selectedRoutine.name);
        populateQuestions(selectedRoutine);
        var questions = selectedRoutine.questions.get();
        for (int i = 0; i < responseChoosers.size(); i++) {
            if(i < questions.size()) {
                var question = questions.get(i);
                questionPublishers.get(i).set(question.name);

                var chooser = responseChoosers.get(i);
                chooser.setOptions(question.getOptionNames());
                var setOption = Optional.ofNullable(question.settingsSupplier.get().defaultOption()).map(Map.Entry::getKey);
                chooser.setDefault(setOption);
                if(
                    lastConfiguration == null ||
                    config.routine() != lastConfiguration.routine() ||
                    lastConfiguration.questions.get(question.name) == SwitchableChooser.placeholder
                ) {
                    chooser.setActive(setOption);
                }

                var response = chooser.get();
                config.addQuestion(question.name, response.orElse(SwitchableChooser.placeholder));
                question.setResponse(response);
            } else {
                questionPublishers.get(i).set(questionPlaceHolder);
                responseChoosers.get(i).setOptions();
            }
        }
        if(!config.equals(lastConfiguration)) {
            System.out.println("[AutoSelector] Generating new command\n" + config);
            lastCommand = AutoManager.generateAutoCommand(selectedRoutine);
        }
        lastConfiguration = config;
        configPublisher.set(lastConfiguration.toString());
    }

    public Command getSelectedAutoCommand() {
        return lastCommand;
    }

    public static record AutoConfiguration(
        Alliance alliance,
        String routine,
        Map<String, String> questions
    ) {
        public AutoConfiguration(Alliance alliance, String routine) {
            this(alliance, routine, new LinkedHashMap<>());
        }
        public void addQuestion(String question, String response) {
            questions.put(question, response);
        }

        public String toString() {
            var builder = new StringBuilder()
                .append("\t").append("Alliance: ").append(alliance).append("\n")
                .append("\t").append("Routine: ").append(routine)
            ;
            questions.entrySet().stream().forEach((e) -> {
                builder.append("\n\t").append(e.getKey()).append(": ").append(e.getValue());
            });
            return builder.toString();
        }
    }

    public static class AutoQuestion<T> {
        public final String name;
        private final Supplier<Settings<T>> settingsSupplier;
        private T response;

        public static record Settings<T>(Map<String, T> options, Map.Entry<String,T> defaultOption) {
            @SafeVarargs
            public static <T> Settings<T> from(Map.Entry<String,T> defaultOption, Map.Entry<String,T>... options) {
                var map = new LinkedHashMap<String, T>();
                for(var option : options) {
                    map.put(option.getKey(), option.getValue());
                }
                return new Settings<T>(map, defaultOption);
            }

            public static <T> Settings<T> empty() {
                return new Settings<T>(Map.of(), null);
            }
        }

        public AutoQuestion(String name, Supplier<Settings<T>> settingsSupplier) {
            this.name = name;
            this.settingsSupplier = settingsSupplier;
            this.response = this.settingsSupplier.get().defaultOption().getValue();
        }

        public T getResponse() {
            return response;
        }

        public void setResponse(Optional<String> newResponse) {
            newResponse
                .map((newR) -> 
                    Optional.ofNullable(
                        settingsSupplier.get().options().get(newR)
                    )
                )
                .orElseGet(() -> 
                    Optional.ofNullable(
                        settingsSupplier.get().defaultOption()
                    ).map(Map.Entry::getValue)
                )
                .ifPresent((r) -> response = r);
            ;
        }

        public String[] getOptionNames() {
            return settingsSupplier.get().options().keySet().stream().toArray(String[]::new);
        }

        public boolean equals(Object obj) {
            if (obj == null || !(obj instanceof AutoQuestion)) return false;

            AutoQuestion<?> other = (AutoQuestion<?>) obj;
            return this.name.equals(other.name) && Arrays.equals(this.getOptionNames(), other.getOptionNames());
        }
    }

    public static abstract class AutoRoutine {
        public final String name;
        public final Supplier<List<AutoQuestion<?>>> questions;

        public AutoRoutine(String name, Supplier<List<AutoQuestion<?>>> questions) {
            this.name = name;
            this.questions = questions;
        }

        public abstract Command generateCommand();
    }
}
