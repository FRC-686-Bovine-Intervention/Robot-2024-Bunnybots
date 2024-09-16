// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.DriveModulePosition;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOFalcon550;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.commands.FieldOrientedDrive;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.manualOverrides.ManualOverrides;
import frc.robot.subsystems.vision.apriltag.ApriltagVision;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import frc.robot.util.controllers.ButtonBoard3x3;
import frc.robot.util.controllers.Joystick;
import frc.robot.util.controllers.XboxController;

public class RobotContainer {
    // Subsystems
    public final Drive drive;
    public final ApriltagVision apriltagVision;
    public final ManualOverrides manualOverrides;
    public final Leds leds;

    // Controllers
    private final XboxController driveController = new XboxController(0);
    private final Joystick driveJoystick;
    private final Supplier<ChassisSpeeds> joystickTranslational;
    private final ButtonBoard3x3 buttonBoard = new ButtonBoard3x3(1);
    private final CommandJoystick simJoystick = new CommandJoystick(2);

    public RobotContainer() {
        System.out.println(
                "[Init RobotContainer] Creating " + RobotType.getMode().name() + " " + RobotType.getRobot().name());

        switch (RobotType.getMode()) {
            case REAL:
                drive = new Drive(
                    new GyroIOPigeon2(),
                    new ModuleIOFalcon550(DriveModulePosition.FRONT_LEFT),
                    new ModuleIOFalcon550(DriveModulePosition.FRONT_RIGHT),
                    new ModuleIOFalcon550(DriveModulePosition.BACK_LEFT),
                    new ModuleIOFalcon550(DriveModulePosition.BACK_RIGHT)
                );
                apriltagVision = new ApriltagVision();
            break;
            case SIM:
                drive = new Drive(
                    new GyroIO() {},
                    new ModuleIOSim(),
                    new ModuleIOSim(),
                    new ModuleIOSim(),
                    new ModuleIOSim()
                );
                apriltagVision = new ApriltagVision();
            break;
            default:
            case REPLAY:
                drive = new Drive(
                    new GyroIO() {},
                    new ModuleIOSim(),
                    new ModuleIOSim(),
                    new ModuleIOSim(),
                    new ModuleIOSim()
                );
                apriltagVision = new ApriltagVision();
            break;
        }
        leds = new Leds();
        manualOverrides = new ManualOverrides();

        driveJoystick = driveController.leftStick
                .smoothRadialDeadband(DriveConstants.driveJoystickDeadbandPercent)
                .radialSensitivity(0.75)
                .radialSlewRateLimit(DriveConstants.joystickSlewRateLimit);

        joystickTranslational = FieldOrientedDrive.joystickSpectatorToFieldRelative(
                driveJoystick,
                () -> false
        );

        System.out.println("[Init RobotContainer] Configuring Default Subsystem Commands");
        configureSubsystems();

        System.out.println("[Init RobotContainer] Configuring Controls");
        configureControls();

        System.out.println("[Init RobotContainer] Configuring Notifications");
        configureNotifications();

        System.out.println("[Init RobotContainer] Configuring Autonomous Modes");
        configureAutos();

        System.out.println("[Init RobotContainer] Configuring System Check");
        configureSystemCheck();

        if (Constants.tuningMode) {
            new Alert("Tuning mode active, do not use in competition.", AlertType.INFO).set(true);
        }
    }

    private void configureSubsystems() {
        drive.translationSubsystem.setDefaultCommand(
                drive.translationSubsystem.fieldRelative(joystickTranslational)
                    .withName("Driver Control Field Relative")
        );
    }

    private void configureControls() {}

    private void configureNotifications() {}

    private void configureAutos() {}

    private void configureSystemCheck() {}

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
