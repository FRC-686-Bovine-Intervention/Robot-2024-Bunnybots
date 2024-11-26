package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.util.LoggedTunableMeasure;
import frc.util.robotStructure.GamepiecePose;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public static final LoggedTunableMeasure<VoltageUnit> intakeVoltage = new LoggedTunableMeasure<>("Intake/Voltages/Intake", Volts.of(4));
    public static final LoggedTunableMeasure<VoltageUnit> holdVoltage = new LoggedTunableMeasure<>("Intake/Voltages/Hold", Volts.of(2));

    public final GamepiecePose gamepiecePose = new GamepiecePose(
        new Transform3d(
            new Translation3d(

            ),
            Rotation3d.kZero
        )
    );

    public final Trigger hasBucket = new Trigger(() -> inputs.proximity.lte(IntakeConstants.sensorThreshold));

    public Intake(IntakeIO io) {
        this.io = io;
        SmartDashboard.putData("Subsystems/Intake", this);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Intake", inputs);


    }

    private Command genCommand(
        String name,
        Supplier<Measure<VoltageUnit>> voltage,
        boolean open
    ) {
        var subsystem = this;
        return new Command() {
            {
                setName(name);
                addRequirements(subsystem);
            }

            @Override
            public void initialize() {
                io.setOpen(open);
            }

            @Override
            public void execute() {
                io.setMotorVoltage(voltage.get());
            }

            @Override
            public void end(boolean interrupted) {
                io.setMotorVoltage(Volts.zero());
                io.setOpen(false);
            }
        };
    }

    public Command idle() {
        return genCommand(
            "Idle",
            () -> (hasBucket.getAsBoolean()) ? (
                holdVoltage.get()
            ) : (
                Volts.zero()
            ),
            false
        );
    }
    public Command intake() {
        return genCommand(
            "Intake",
            intakeVoltage,
            true
        );
    }
    public Command eject() {
        return genCommand(
            "Eject",
            Volts::zero,
            true
        );
    }
}
