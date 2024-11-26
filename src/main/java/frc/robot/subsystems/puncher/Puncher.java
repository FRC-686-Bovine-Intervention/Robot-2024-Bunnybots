package frc.robot.subsystems.puncher;

import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.robotStructure.GamepiecePose;
import frc.util.robotStructure.linear.ExtenderMech;

public class Puncher extends SubsystemBase {
    private final PuncherIO io;
    private final PuncherIOInputsAutoLogged inputs = new PuncherIOInputsAutoLogged();

    public final ExtenderMech mech = new ExtenderMech(
        new Transform3d(
            new Translation3d(

            ),
            Rotation3d.kZero
        )
    );
    public final GamepiecePose gamepiecePunch = new GamepiecePose(
        new Transform3d(
            new Translation3d(

            ),
            Rotation3d.kZero
        )
    );

    public Puncher(PuncherIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Puncher", inputs);
    }

    public Command genCommand(
        String name,
        boolean deployed
    ) {
        var subsystem = this;
        return new Command() {
            {
                setName(name);
                addRequirements(subsystem);
            }

            @Override
            public void initialize() {
                io.setDeployed(deployed);
                mech.set(
                    (deployed) ? (
                        PuncherConstants.punchDistance
                    ) : (
                        Meters.zero()
                    )
                );
            }

            @Override
            public void end(boolean interrupted) {
                io.setDeployed(false);
                mech.set(Meters.zero());
            }
        };
    }

    public Command retract() {
        return genCommand(
            "Retract",
            false
        );
    }
    public Command punch() {
        return genCommand(
            "Deploy",
            true
        );
    }
}
