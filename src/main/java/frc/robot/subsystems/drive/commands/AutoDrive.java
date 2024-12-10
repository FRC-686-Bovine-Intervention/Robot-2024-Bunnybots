package frc.robot.subsystems.drive.commands;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.util.AllianceFlipUtil;
import frc.util.MathExtraUtil;

public class AutoDrive {
    public static Command preciseToPose(Pose2d pose, Drive drive) {
        return new Command() {
            {
                addRequirements(drive.subsystems);
            }
            private final PIDController xController = new PIDController(2, 0, 0);
            private final PIDController yController = new PIDController(15, 0, 0);
            private final PIDController thetaController = new PIDController(3, 0, 0);

            @Override
            public void initialize() {
                xController.reset();
                yController.reset();
                thetaController.reset();
            }

            @Override
            public void execute() {
                var currentPose = drive.getPose();
                var xOut = xController.calculate(currentPose.getX(), pose.getX());
                var yOut = yController.calculate(currentPose.getY(), pose.getY());
                var thetaOut = thetaController.calculate(currentPose.getRotation().minus(pose.getRotation()).getRadians(), 0);

                drive.runFieldSpeeds(new ChassisSpeeds(
                    xOut,
                    yOut,
                    thetaOut
                ));
            }

            @Override
            public void end(boolean interrupted) {
                drive.stop();
            }
        };
    }
    public static BooleanSupplier withinTolerance(Pose2d pose, Drive drive) {
        return () -> MathExtraUtil.isNear(pose, drive.getPose(), Centimeters.of(2), Degrees.of(10));
    }
    public static Command autoDriveToHighGoal(Drive drive) {
        return Commands.defer(
            new Supplier<Command>() {
                @Override
                public Command get() {
                    var currentPose = drive.getPose();
                    var points = new ArrayList<Pose2d>(5);
                    var rotationTargets = new ArrayList<RotationTarget>(2);
                    var constraintZones = new ArrayList<ConstraintsZone>(1);
                    var currentBluePose = AllianceFlipUtil.apply(currentPose);
                    points.add(new Pose2d(
                        currentPose.getTranslation(),
                        new Rotation2d(
                            -drive.getFieldMeasuredSpeeds().vxMetersPerSecond,
                            -drive.getFieldMeasuredSpeeds().vyMetersPerSecond
                        )
                    ));
                    var includePreEntry = currentBluePose.getMeasureX().gt(FieldConstants.denPreEntryX);
                    var includeEntry = currentBluePose.getMeasureX().gt(FieldConstants.denEntryX) && currentBluePose.getMeasureY().lte(FieldConstants.topObsticalBottomEdgeY);
                    var useFieldEntry = currentBluePose.getMeasureY().gt(FieldConstants.denEntryDecisionY);
                    var goal = (useFieldEntry) ? (
                        FieldConstants.highGoalScoreStackingSide.getOurs()
                    ) : (
                        FieldConstants.highGoalScoreSourceSide.getOurs()
                    );
                    if (includePreEntry) {
                        points.add(
                            (useFieldEntry) ? (
                                FieldConstants.denFieldPreEntry.getOurs()
                            ) : (
                                FieldConstants.denSourcePreEntry.getOurs()
                            )
                        );
                        rotationTargets.add(new RotationTarget(
                            points.size() - 1,
                            FieldConstants.denSourcePreEntry.getOurs().getRotation()
                        ));
                    }
                    if (includeEntry) {
                        points.add(
                            (useFieldEntry) ? (
                                FieldConstants.denFieldEntry.getOurs()
                            ) : (
                                FieldConstants.denSourceEntry.getOurs()
                            )
                        );
                        rotationTargets.add(new RotationTarget(
                            points.size() - 1,
                            FieldConstants.denSourceEntry.getOurs().getRotation()
                        ));
                    }
                    points.add(
                        (useFieldEntry) ? (
                            FieldConstants.highGoalPreScoreStackingSide.getOurs()
                        ) : (
                            FieldConstants.highGoalPreScoreSourceSide.getOurs()
                        )
                    );
                    points.add(goal);
                    // constraintZones.add(new ConstraintsZone(
                    //     points.size() - 2,
                    //     points.size() - 1,
                    //     new PathConstraints(
                    //         1,
                    //         1,
                    //         1,
                    //         1
                    //     )
                    // ));
                    return drive.followPath(new PathPlannerPath(
                        PathPlannerPath.waypointsFromPoses(points),
                        rotationTargets,
                        Collections.emptyList(),
                        constraintZones,
                        Collections.emptyList(),
                        new PathConstraints(
                            3,
                            3,
                            3,
                            3
                        ),
                        null,
                        new GoalEndState(0, FieldConstants.highGoalScoreStackingSide.getOurs().getRotation()),
                        false
                    ))
                    .andThen(preciseToPose(goal, drive))
                    ;
                }
            },
            drive.subsystems
        );
    }
    public static Command autoDriveToStackingGrid(Drive drive) {
        return Commands.defer(
            new Supplier<Command>() {
                @Override
                public Command get() {
                    var currentPose = drive.getPose();
                    var points = new ArrayList<Pose2d>(5);
                    var rotationTargets = new ArrayList<RotationTarget>(2);
                    var constraintZones = new ArrayList<ConstraintsZone>(1);
                    var currentBluePose = AllianceFlipUtil.apply(currentPose);
                    points.add(new Pose2d(
                        currentPose.getTranslation(),
                        new Rotation2d(
                            -drive.getFieldMeasuredSpeeds().vxMetersPerSecond,
                            -drive.getFieldMeasuredSpeeds().vyMetersPerSecond
                        )
                    ));
                    var includePreEntry = currentBluePose.getMeasureX().gt(FieldConstants.denPreEntryX);
                    var includeEntry = currentBluePose.getMeasureX().gt(FieldConstants.denEntryX) && currentBluePose.getMeasureY().lte(FieldConstants.denFieldEntryY);
                    var useFieldEntry = currentBluePose.getMeasureY().gt(FieldConstants.denEntryDecisionY);
                    var goal = new Pose2d(
                        new Translation2d(
                            FieldConstants.stackingScoreX.in(Meters),
                            MathUtil.clamp(
                                points.get(points.size()-1).getY(),
                                FieldConstants.stackingMinY.in(Meters),
                                FieldConstants.stackingMaxY.in(Meters)
                            )
                        ),
                        FieldConstants.highGoalScoreStackingSide.getOurs().getRotation()
                    );
                    if (includePreEntry) {
                        points.add(
                            (useFieldEntry) ? (
                                FieldConstants.denFieldPreEntry.getOurs()
                            ) : (
                                FieldConstants.denSourcePreEntry.getOurs()
                            )
                        );
                        rotationTargets.add(new RotationTarget(
                            points.size() - 1,
                            FieldConstants.denSourcePreEntry.getOurs().getRotation()
                        ));
                    }
                    if (includeEntry) {
                        points.add(
                            (useFieldEntry) ? (
                                FieldConstants.denFieldEntry.getOurs()
                            ) : (
                                FieldConstants.denSourceEntry.getOurs()
                            )
                        );
                        rotationTargets.add(new RotationTarget(
                            points.size() - 1,
                            FieldConstants.denSourceEntry.getOurs().getRotation()
                        ));
                    }
                    points.add(goal);
                    // constraintZones.add(new ConstraintsZone(
                    //     points.size() - 2,
                    //     points.size() - 1,
                    //     new PathConstraints(
                    //         1,
                    //         1,
                    //         1,
                    //         1
                    //     )
                    // ));
                    return drive.followBluePath(new PathPlannerPath(
                        PathPlannerPath.waypointsFromPoses(points),
                        rotationTargets,
                        Collections.emptyList(),
                        constraintZones,
                        Collections.emptyList(),
                        new PathConstraints(
                            3,
                            3,
                            3,
                            3
                        ),
                        null,
                        new GoalEndState(0, FieldConstants.highGoalScoreStackingSide.getOurs().getRotation()),
                        false
                    ))
                    .andThen(preciseToPose(goal, drive))
                    ;
                }
            },
            Set.of(drive.translationSubsystem, drive.rotationalSubsystem)
        );
    }
}
