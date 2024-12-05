// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.AutoManager;
import frc.robot.auto.AutoSelector;
import frc.robot.auto.AutoCommons.AutoPaths;
import frc.robot.auto.AutoSelector.AutoRoutine;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOFalcon;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOFalcon550;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.commands.AutoDrive;
import frc.robot.subsystems.drive.commands.DriveToPose;
import frc.robot.subsystems.drive.commands.WheelRadiusCalibration;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalon;
import frc.robot.subsystems.manualOverrides.ManualOverrides;
import frc.robot.subsystems.puncher.Puncher;
import frc.robot.subsystems.puncher.PuncherIO;
import frc.robot.subsystems.puncher.PuncherIOSolenoid;
import frc.robot.subsystems.vision.apriltag.ApriltagVision;
import frc.util.controllers.ButtonBoard3x3;
import frc.util.controllers.Joystick;
import frc.util.controllers.XboxController;
import frc.util.robotStructure.Mechanism3d;

public class RobotContainer {
    // Subsystems
    public final Drive drive;
    public final Arm arm;
    public final Intake intake;
    public final Puncher puncher;
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
                arm = new Arm(new ArmIOFalcon());
                intake = new Intake(new IntakeIOTalon());
                puncher = new Puncher(new PuncherIOSolenoid());
            break;
            case SIM:
                drive = new Drive(
                    new GyroIO() {},
                    Arrays.stream(DriveConstants.moduleConstants)
                        .map(ModuleIOSim::new)
                        .toArray(ModuleIO[]::new)
                );
                apriltagVision = new ApriltagVision();
                arm = new Arm(new ArmIOSim());
                intake = new Intake(new IntakeIOSim());
                puncher = new Puncher(new PuncherIO() {});
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
                arm = new Arm(new ArmIO(){});
                intake = new Intake(new IntakeIO() {});
                puncher = new Puncher(new PuncherIO() {});
            break;
        }
        manualOverrides = new ManualOverrides();

        drive.structureRoot
            .addChild(arm.mech
                .addChild(intake.gamepiecePose)
                .addChild(intake.leftClaw)
                .addChild(intake.rightClaw)
            )
            .addChild(puncher.mech
                .addChild(puncher.gamepiecePunch)
            )
        ;
        Mechanism3d.registerMechs(arm.mech, puncher.mech, intake.leftClaw, intake.rightClaw);

        driveJoystick = driveController.leftStick
            .smoothRadialDeadband(DriveConstants.driveJoystickDeadbandPercent)
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
            new Alert("Tuning mode active.", AlertType.kInfo).set(true);
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
        arm.setDefaultCommand(
            arm.idle()
        );
        intake.setDefaultCommand(
            intake.idle()
        );
        puncher.setDefaultCommand(
            puncher.retract()
        );
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


        driveController.a().toggleOnTrue(
            Commands.parallel(
                arm.floor(),
                intake.intake()
            )
            .until(intake.hasBucket)
            .andThen(
                Commands.parallel(
                    arm.floor(),
                    intake.idle()
                )
                .withTimeout(0.5)
            )
        );

        intake.hasBucket.whileTrue(arm.puncher());
        driveController.x().whileTrue(
            Commands.parallel(
                intake.eject(),
                puncher.punch()
            )
        );
        driveController.b().toggleOnTrue(
            Commands.parallel(
                arm.floor()
                // intake.eject()
            )
        );
        driveController.y().whileTrue(
            Commands.parallel(
                intake.eject()
            )
        );

        driveController.leftStickButton().onTrue(Commands.runOnce(() -> {
            drive.setPose(FieldConstants.highGoalScoreStackingSide.getOurs());
        }));
        // driveController.rightBumper().whileTrue(new DriveToPose(drive, FieldConstants.highGoalStackingSide::getOurs));
        driveController.rightBumper().whileTrue(AutoDrive.autoDriveToHighGoal(drive));
        SmartDashboard.putData("Reset Pose", Commands.runOnce(() -> {
            drive.setPose(new Pose2d());
        }));
        SmartDashboard.putData("Arm/Coast", arm.coast());
        SmartDashboard.putData("Arm/Voltage", arm.voltage(
            () -> 2 * (driveController.leftTrigger.getAsDouble() - driveController.rightTrigger.getAsDouble())
        ));
    }

    private void configureNotifications() {}

    private void configureAutos() {
        AutoPaths.preload();
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
        SmartDashboard.putData("System Check/Arm/Puncher", arm.getDefaultCommand());
        SmartDashboard.putData("System Check/Arm/Floor", arm.floor());
        
        SmartDashboard.putData("System Check/Intake/Hold", intake.getDefaultCommand());
        SmartDashboard.putData("System Check/Intake/Intake", intake.intake());
        SmartDashboard.putData("System Check/Intake/Eject", intake.eject());
        
        SmartDashboard.putData("System Check/Puncher/Reject", puncher.getDefaultCommand());
        SmartDashboard.putData("System Check/Puncher/Punch", puncher.punch());

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
        
        SmartDashboard.putData("Wheel Calibration", Commands.defer(() -> 
            new WheelRadiusCalibration(
                drive,
                (int)WheelRadiusCalibration.MAX_SAMPLES.get(),
                WheelRadiusCalibration.SAMPLE_PERIOD.get(),
                WheelRadiusCalibration.VOLTAGE_RAMP_RATE.get(),
                WheelRadiusCalibration.MAX_VOLTAGE.get()
            ).withName("Wheel Calibration"),
            Set.of(drive.translationSubsystem, drive.rotationalSubsystem))
        );
    }
}
