// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.util.Cooldown;
import frc.util.LoggedInternalButton;
import frc.util.LoggedTunableMeasure;
import frc.util.MathExtraUtil;
import frc.util.robotStructure.Mechanism3d;
import frc.util.robotStructure.angle.ArmMech;

public class Arm extends SubsystemBase {
    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    public final ArmMech mech = new ArmMech(
        new Transform3d(
            new Translation3d(
                Inches.of(5.625000),
                Inches.of(0),
                Inches.of(14.250000)
            ),
            Rotation3d.kZero
        )
    );

    public static final LoggedTunableMeasure<AngleUnit> lowGoalSetpoint = new LoggedTunableMeasure<>("Arm/Setpoints/Low Goal", Degrees.of(30));
    public static final LoggedTunableMeasure<AngleUnit> floorSetpoint = new LoggedTunableMeasure<>("Arm/Setpoints/Floor", Degrees.of(0));
    public static final LoggedTunableMeasure<AngleUnit> puncherSetpoint = new LoggedTunableMeasure<>("Arm/Setpoints/Puncher", Degrees.of(95));
    
    private final LoggedTunableMeasure<AngleUnit> customAngleIncrement = new LoggedTunableMeasure<>("Arm/Setpoints/Custom/Increment", Degrees.of(5));

    public final LoggedInternalButton atPos = new LoggedInternalButton("Arm/At Position");

    public Arm(ArmIO io) {
        System.out.println("[Init Arm] Instantiating Arm");
        this.io = io;
        System.out.println("[Init Arm] Arm IO: " + this.io.getClass().getSimpleName());
        SmartDashboard.putData("Subsystems/Arm", this);

        var routine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> {
                    Logger.recordOutput("SysID/Arm/State", state.toString());
                }
            ),
            new SysIdRoutine.Mechanism(
                (volts) -> {
                    io.setVoltage(volts);
                },
                (log) -> {
                    Logger.recordOutput("SysID/Arm/Position", inputs.encoder.position);
                    Logger.recordOutput("SysID/Arm/Velocity", inputs.encoder.velocity);
                    Logger.recordOutput("SysID/Arm/Voltage", inputs.motor.appliedVoltage);
                },
                this
            )
        );

        SmartDashboard.putData("SysID/Arm/Quasi Forward", routine.quasistatic(Direction.kForward).until(() -> inputs.encoder.position.gte(ArmConstants.maxAngle)).withName("SysID Quasistatic Forward").asProxy());
        SmartDashboard.putData("SysID/Arm/Quasi Reverse", routine.quasistatic(Direction.kReverse).until(() -> inputs.encoder.position.lte(ArmConstants.minAngle)).withName("SysID Quasistatic Reverse").asProxy());
        SmartDashboard.putData("SysID/Arm/Dynamic Forward", routine.dynamic(Direction.kForward).until(() -> inputs.encoder.position.gte(ArmConstants.maxAngle)).withName("SysID Dynamic Forward").asProxy());
        SmartDashboard.putData("SysID/Arm/Dynamic Reverse", routine.dynamic(Direction.kReverse).until(() -> inputs.encoder.position.lte(ArmConstants.minAngle)).withName("SysID Dynamic Reverse").asProxy());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Arm", inputs);

        mech.set(inputs.encoder.position);
        mech.log(Mechanism3d.KEY + "/Arm");
    }

    public Command genCommand(String name, Supplier<Measure<AngleUnit>> posSupplier) {
        var subsystem = this;
        return new Command() {
            {
                setName(name);
                addRequirements(subsystem);
            }

            @Override
            public void initialize() {
                execute();
            }

            @Override
            public void execute() {
                atPos.setPressed(MathExtraUtil.isNear(
                    posSupplier.get(),
                    inputs.encoder.position,
                    ArmConstants.tolerance.get()
                ));
                io.setAngle(posSupplier.get()); 
            }

            @Override
            public void end(boolean interrupted) {
                io.stop();
            }
        };
    }

    public Command floor() {
        return genCommand(
            "Floor",
            floorSetpoint
        );
    }

    public Command puncher() {
        return genCommand(
            "Puncher",
            puncherSetpoint
        );
    }

    public Command lowGoal() {
        return genCommand(
            "Low Goal",
            lowGoalSetpoint
        );
    }

    public Command customVariable(BooleanSupplier increase, BooleanSupplier decrease) {
        return genCommand("Custom", new Supplier<Measure<AngleUnit>>() {
            private final MutAngle pos = Degrees.mutable(0);
            private final Cooldown cooldown = new Cooldown(0.125);
            
            public Measure<AngleUnit> get() {
                if(!cooldown.hasExpired()) {
                    return pos;
                }
                if(increase.getAsBoolean()) {
                    cooldown.reset();
                    pos.mut_acc(customAngleIncrement.in(Degrees));
                }
                if(decrease.getAsBoolean() && pos.gt(Degrees.zero())) {
                    cooldown.reset();
                    pos.mut_acc(-customAngleIncrement.in(Degrees));
                }
                Logger.recordOutput("Arm/Custom Setpoint", pos);
                return pos;
            }
        });
    }

    public Command coast() {
        var subsystem = this;
        return new Command() {
            {
                setName("Coast");
                addRequirements(subsystem);
            }

            @Override
            public void initialize() {
                io.setCoast(true);
            }

            @Override
            public void end(boolean interrupted) {
                io.setCoast(false);
            }

            @Override
            public boolean runsWhenDisabled() {
                return true;
            }
        };
    }

    public Command voltage(DoubleSupplier voltage) {
        var subsystem = this;
        return new Command() {
            {
                setName("Voltage");
                addRequirements(subsystem);
            }

            @Override
            public void initialize() {

            }

            @Override
            public void execute() {
                io.setVoltage(Volts.of(voltage.getAsDouble()));
            }

            @Override
            public void end(boolean interrupted) {
                io.setCoast(false);
            }

            @Override
            public boolean runsWhenDisabled() {
                return true;
            }
        };
    }
}
