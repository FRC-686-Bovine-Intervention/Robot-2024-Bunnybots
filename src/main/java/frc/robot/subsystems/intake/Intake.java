package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.util.LoggedTunableMeasure;
import frc.util.robotStructure.GamepiecePose;
import frc.util.robotStructure.angle.TurretMech;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public static final LoggedTunableMeasure<TimeUnit> intakeConfirmTime = new LoggedTunableMeasure<>("Intake/Intake Confirm Time", Seconds.of(0.5));
    public static final LoggedTunableMeasure<VoltageUnit> slowIntakeVoltage = new LoggedTunableMeasure<>("Intake/Voltages/Slow Intake", Volts.of(4));
    public static final LoggedTunableMeasure<VoltageUnit> fastIntakeVoltage = new LoggedTunableMeasure<>("Intake/Voltages/Fast Intake", Volts.of(8));
    public static final LoggedTunableMeasure<VoltageUnit> holdVoltage = new LoggedTunableMeasure<>("Intake/Voltages/Hold", Volts.of(2));

    public final TurretMech leftClaw = new TurretMech(
        new Transform3d(
            new Translation3d(
                Meters.of(+0.334556),
                Meters.of(+0.267493),
                Meters.of(-0.102273)
            ),
            Rotation3d.kZero
        )
    );
    public final TurretMech rightClaw = new TurretMech(
        new Transform3d(
            new Translation3d(
                Meters.of(+0.334556),
                Meters.of(-0.267493),
                Meters.of(-0.102273)
            ),
            Rotation3d.kZero
        )
    );

    public final GamepiecePose gamepiecePose = new GamepiecePose(
        new Transform3d(
            new Translation3d(
                Inches.of(20.750000),
                Inches.of(0),
                Inches.of(0.250000)
            ),
            new Rotation3d(Math.PI, 0, 0)
        )
    );

    public final Trigger hasBucket = new Trigger(() -> inputs.sensorDetect);

    public Intake(IntakeIO io) {
        this.io = io;
        SmartDashboard.putData("Subsystems/Intake", this);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Intake", inputs);

        Logger.recordOutput("Intake/Gamepiece",
            (hasBucket.getAsBoolean()) ? (
                new Pose3d[]{
                    gamepiecePose.getFieldRelative()
                }
            ) : (
                new Pose3d[]{}
            )
        );
    }

    private void setClawOpen(boolean open) {
        io.setOpen(open);
        leftClaw.set(
            (open) ? (
                Radians.zero()
            ) : (
                IntakeConstants.clawMotion.unaryMinus()
            )
        );
        rightClaw.set(
            (open) ? (
                Radians.zero()
            ) : (
                IntakeConstants.clawMotion
            )
        );
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
                setClawOpen(open);
            }

            @Override
            public void execute() {
                io.setMotorVoltage(voltage.get());
            }

            @Override
            public void end(boolean interrupted) {
                io.setMotorVoltage(Volts.zero());
                setClawOpen(false);
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
    public Command eject() {
        return genCommand(
            "Eject",
            Volts::zero,
            true
        );
    }
    public Command intake(BooleanSupplier open) {
        var subsystem = this;
        return new Command() {
            {
                setName("Intake");
                addRequirements(subsystem);
            }

            private final Timer sensorTimer = new Timer();

            @Override
            public void initialize() {
                
            }

            @Override
            public void execute() {
                if (inputs.sensorDetect) {
                    sensorTimer.start();
                } else {
                    sensorTimer.stop();
                    sensorTimer.reset();
                }
                var openValue = open.getAsBoolean();
                setClawOpen(openValue && !inputs.sensorDetect);
                io.setMotorVoltage(
                    (openValue) ? (
                        slowIntakeVoltage.get()
                    ) : (
                        fastIntakeVoltage.get()
                    )
                );
            }

            @Override
            public void end(boolean interrupted) {
                sensorTimer.stop();
                sensorTimer.reset();
                io.setMotorVoltage(Volts.zero());
                setClawOpen(false);
            }

            @Override
            public boolean isFinished() {
                return sensorTimer.hasElapsed(intakeConfirmTime.in(Seconds));
            }
        };
    }
}
