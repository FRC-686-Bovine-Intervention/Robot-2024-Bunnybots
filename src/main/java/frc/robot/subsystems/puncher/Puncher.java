// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.puncher;

import static edu.wpi.first.units.Units.Inches;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.LoggedInternalButton;
import frc.util.robotStructure.Mechanism3d;
import frc.util.robotStructure.linear.ExtenderMech;

public class Puncher extends SubsystemBase {
    private final PuncherIO io;
    private final PuncherIOInputsAutoLogged inputs = new PuncherIOInputsAutoLogged();

    private final LoggedInternalButton isExtended = new LoggedInternalButton("Puncher/Extended");

    public final ExtenderMech mech = new ExtenderMech(
        new Transform3d(new Translation3d(
            Inches.of(0),
            Inches.of(12.875000),
            Inches.of(18.750000)
        ),
        Rotation3d.kZero
    ));

    public Puncher(PuncherIO io) {
		System.out.println("[Init Puncher] Instantiating Puncher");
        this.io = io;
        System.out.println("[Init Puncher] Puncher IO: " + this.io.getClass().getSimpleName());
        SmartDashboard.putData("Subsystems/Puncher", this);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Puncher", inputs);

        mech.set(isExtended.getAsBoolean() ? Inches.of(11.497488) : Inches.of(0));
        mech.log(Mechanism3d.KEY + "/Puncher");
    }

    public Command extend() {
        var subsystem = this;
        return new Command() {
            {
                addRequirements(subsystem);
            }

            @Override
            public void initialize() {
                execute();
            }

            @Override
            public void execute() {
                puncherExtend();
            }
            
            @Override
            public void end(boolean interrupted) {
                puncherRetract();
            }
        };
    }

    public Command retract() {
        var subsystem = this;
        return new Command() {
            {
                addRequirements(subsystem);
            }

            @Override
            public void initialize() {
                execute();
            }

            @Override
            public void execute() {
                puncherRetract();
            }
        };
    }

    private void puncherExtend() {
        isExtended.setPressed(true);
        io.extend();
    }

    private void puncherRetract() {
        isExtended.setPressed(false);
        io.retract();
    }
}
