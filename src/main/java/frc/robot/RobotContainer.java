// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.AutoManager;
import frc.robot.auto.AutoSelector;
import frc.robot.auto.AutoSelector.AutoRoutine;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOFalcon550;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.manualOverrides.ManualOverrides;
import frc.robot.subsystems.vision.apriltag.ApriltagVision;
import frc.util.Alert;
import frc.util.Alert.AlertType;
import frc.util.controllers.ButtonBoard3x3;
import frc.util.controllers.Joystick;
import frc.util.controllers.XboxController;

public class RobotContainer {
    // Subsystems
    public final Drive drive;
    public final ApriltagVision apriltagVision;
    public final ManualOverrides manualOverrides;

    // Controllers
    private final XboxController driveController = new XboxController(0);
    private final Joystick driveJoystick;
    private final Supplier<ChassisSpeeds> joystickTranslational;
    private final ButtonBoard3x3 buttonBoard = new ButtonBoard3x3(1);
    private final CommandJoystick simJoystick = new CommandJoystick(2);

    public RobotContainer() {
        System.out.println("[Init RobotContainer] Creating " + RobotType.getMode().name() + " " + RobotType.getRobot().name());

        switch (RobotType.getMode()) {
            case REAL:
                drive = new Drive(
                    new GyroIOPigeon2(),
                    Arrays.stream(DriveConstants.moduleConstants)
                        .map(ModuleIOFalcon550::new)
                        .toArray(ModuleIO[]::new)
                );
                apriltagVision = new ApriltagVision();
            break;
            case SIM:
                drive = new Drive(
                    new GyroIO() {},
                    Arrays.stream(DriveConstants.moduleConstants)
                        .map(ModuleIOSim::new)
                        .toArray(ModuleIO[]::new)
                );
                apriltagVision = new ApriltagVision();
            break;
            default:
            case REPLAY:
                drive = new Drive(
                    new GyroIO() {},
                    new ModuleIO(){},
                    new ModuleIO(){},
                    new ModuleIO(){},
                    new ModuleIO(){}
                );
                apriltagVision = new ApriltagVision();
            break;
        }
        manualOverrides = new ManualOverrides();

        driveJoystick = driveController.leftStick
            .roughRadialDeadband(DriveConstants.driveJoystickDeadbandPercent)
            .radialSensitivity(0.75)
            // .radialSlewRateLimit(DriveConstants.joystickSlewRateLimit)
        ;

        joystickTranslational = Drive.Translational.joystickSpectatorToFieldRelative(
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

        if (RobotConstants.tuningMode) {
            new Alert("Tuning mode active, do not use in competition.", AlertType.INFO).set(true);
        }
    }

    private void configureSubsystems() {
        drive.translationSubsystem.setDefaultCommand(
            drive.translationSubsystem.fieldRelative(joystickTranslational)
                .withName("Driver Control Field Relative")
        );
        // drive.rotationalSubsystem.setDefaultCommand(
        //     drive.rotationalSubsystem.spin(driveController.rightStick.x().smoothDeadband(0.05).multiply(DriveConstants.maxTurnRate.in(RadiansPerSecond)))
        //         .withName("Robot spin")
        // );
    }

    private void configureControls() {
        driveController.leftStickButton().onTrue(Commands.runOnce(() -> drive.setPose(Pose2d.kZero)));
        var flickStick = driveController.rightStick.roughRadialDeadband(0.85);
        new Trigger(() -> flickStick.magnitude() > 0 && drive.rotationalSubsystem.getCurrentCommand() == null).onTrue(
            drive.rotationalSubsystem.headingFromJoystick(
                flickStick,
                new Rotation2d[]{
                    // Cardinals
                    Rotation2d.kZero,
                    Rotation2d.kCCW_90deg,
                    Rotation2d.k180deg,
                    Rotation2d.kCW_90deg,
                },
                () -> RobotConstants.intakeForward
            )
            .withName("Flick Stick")
        );
    }

    private void configureNotifications() {}

    private void configureAutos() {
        var selector = new AutoSelector("AutoSelector");

        selector.addRoutine(new AutoRoutine("Finish in 5", List.of()) {
            public Command generateCommand() {
                return Commands.waitSeconds(5);
            }
        });
        selector.addRoutine(new AutoRoutine("Finish in 10", List.of()) {
            public Command generateCommand() {
                return Commands.waitSeconds(10);
            }
        });
        selector.addRoutine(new AutoRoutine("Finish in 20", List.of()) {
            public Command generateCommand() {
                return Commands.waitSeconds(20);
            }
        });

        new AutoManager(selector);
    }

    private void configureSystemCheck() {
        SmartDashboard.putData("System Check/Drive/Spin", 
            new Command() {
                private final Drive.Rotational rotationalSubsystem = drive.rotationalSubsystem;
                private final Timer timer = new Timer();
                {
                    addRequirements(rotationalSubsystem);
                    setName("TEST Spin");
                }
                public void initialize() {
                    timer.restart();
                }
                public void execute() {
                    rotationalSubsystem.driveVelocity(Math.sin(timer.get()) * 3);
                }
                public void end(boolean interrupted) {
                    timer.stop();
                    rotationalSubsystem.stop();
                }
            }
        );
        SmartDashboard.putData("System Check/Drive/Circle", 
            new Command() {
                private final Drive.Translational translationSubsystem = drive.translationSubsystem;
                private final Timer timer = new Timer();
                {
                    addRequirements(translationSubsystem);
                    setName("TEST Circle");
                }
                public void initialize() {
                    timer.restart();
                }
                public void execute() {
                    translationSubsystem.driveVelocity(
                        new ChassisSpeeds(
                            Math.cos(timer.get()) * 0.01,
                            Math.sin(timer.get()) * 0.01,
                            0
                        )
                    );
                }
                public void end(boolean interrupted) {
                    timer.stop();
                    translationSubsystem.stop();
                }
            }
        );
    }
}
