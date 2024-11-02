package frc.robot.util;

import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class BranchingRaceCommandGroup extends Command {
    private final HashMap<Command, Command> m_commands = new HashMap<>();
    private boolean m_runWhenDisabled = true;
    private Optional<Command> finishingCommand = Optional.empty();
    private InterruptionBehavior m_interruptBehavior = InterruptionBehavior.kCancelIncoming;

    /**
     * Creates a new ParallelCommandRace. The given commands will be executed simultaneously, and will
     * "race to the finish" - the first command to finish ends the entire command, with all other
     * commands being interrupted.
     *
     * @param commands the commands to include in this composition.
     */
    public BranchingRaceCommandGroup(HashMap<Command, Command> commands) {
        addCommands(commands);
    }

    /**
     * Adds the given commands to the group.
     *
     * @param commands Commands to add to the group.
     */
    public final void addCommands(HashMap<Command, Command> commands) {
        // if (!m_finished) {
        //     throw new IllegalStateException(
        //         "Commands cannot be added to a composition while it's running!");
        // }
        var flatCommands = commands.entrySet().stream().flatMap((entry) -> Arrays.stream(new Command[]{entry.getKey(), entry.getValue()})).toArray(Command[]::new);

        CommandScheduler.getInstance().registerComposedCommands(flatCommands);

        for (Command command : flatCommands) {
            if (!Collections.disjoint(command.getRequirements(), getRequirements())) {
                throw new IllegalArgumentException(
                    "Multiple commands in a parallel composition cannot require the same subsystems");
            }
            getRequirements().addAll(command.getRequirements());
            m_runWhenDisabled &= command.runsWhenDisabled();
            if (command.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf) {
                m_interruptBehavior = InterruptionBehavior.kCancelSelf;
            }
        }

        m_commands.putAll(commands);
    }

    @Override
    public final void initialize() {
        finishingCommand = Optional.empty();
        for (Command command : m_commands.keySet()) {
            command.initialize();
        }
    }

    @Override
    public final void execute() {
        if (finishingCommand.isPresent()) {
            var command = finishingCommand.get();
            if (command.isFinished()) {
                command.end(false);
                return;
            }
            command.execute();
        } else {
            for (Command key : m_commands.keySet()) {
                if (key.isFinished()) {
                    var finishCommand = m_commands.get(key);
                    finishingCommand = Optional.of(finishCommand);
                    key.end(false);
                    m_commands.keySet().stream().filter((command) -> key != command).forEach((command) -> command.end(true));
                    finishCommand.initialize();
                    break;
                }
                key.execute();
            }
        }
    }

    @Override
    public final void end(boolean interrupted) {
        if (finishingCommand.isPresent()) {
            var command = finishingCommand.get();
            command.end(!command.isFinished());
        } else {
            for (Command command : m_commands.keySet()) {
                command.end(!command.isFinished());
            }
        }
    }

    @Override
    public final boolean isFinished() {
        return finishingCommand.filter((command) -> command.isFinished()).isPresent();
    }

    @Override
    public boolean runsWhenDisabled() {
        return m_runWhenDisabled;
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return m_interruptBehavior;
    }
}
