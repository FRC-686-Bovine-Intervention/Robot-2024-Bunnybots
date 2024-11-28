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
                Meters.of(-0.206375),
                Meters.of(+0),
                Meters.of(+0.469900)
            ),
            Rotation3d.kZero
        )
    );
    public final GamepiecePose gamepiecePunch = new GamepiecePose(
        new Transform3d(
            new Translation3d(
                Meters.of(+0.333375),
                Meters.of(+0),
                Meters.of(+0.419100)
            ),
            new Rotation3d(0, Math.PI / 2, 0)
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

    private void setDeployed(boolean deployed) {
        io.setDeployed(deployed);
        mech.set(
            (deployed) ? (
                PuncherConstants.punchDistance
            ) : (
                Meters.zero()
            )
        );
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
                setDeployed(deployed);
            }

            @Override
            public void end(boolean interrupted) {
                setDeployed(false);
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
