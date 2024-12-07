package frc.robot.auto;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.commands.AutoDrive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.puncher.Puncher;
import frc.util.AllianceFlipUtil;
import frc.util.AllianceFlipUtil.FlippedPose2d;

public class AutoCommons {
    public interface ToEntry<E extends Enum<E> & ToEntry<E>> {
        @SuppressWarnings("unchecked")
        default Map.Entry<String, E> toEntry() {
            return Map.entry(this.name(), (E) this);
        }

        String name();
    }

    public static enum StartPosition implements ToEntry<StartPosition> {
        Inner(AutoConstants.innerStartPose),
        Outer(AutoConstants.outerStartPose),
        ;
        public final FlippedPose2d startPose;
        StartPosition(FlippedPose2d startPose) {
            this.startPose = startPose;
        }
    }

    public static enum PathType implements ToEntry<PathType> {
        Close,
        Far
    }

    public static enum Bucket implements ToEntry<Bucket> {
        Bucket1(1),
        Bucket2(2),
        Bucket3(3),
        Bucket4(4),
        ;
        private int number;
        Bucket(int number) {
            this.number = number;
        }
        @Override
        public Entry<String, Bucket> toEntry() {
            return Map.entry(Integer.toString(number), this);
        }
        @Override
        public String toString() {
            return Integer.toString(this.number);
        }
    }

    public static Translation2d getFirstPoint(PathPlannerPath path) {
        return AllianceFlipUtil.apply(path.getPoint(0).position);
    }

    public static Translation2d getLastPoint(PathPlannerPath path) {
        return AllianceFlipUtil.apply(path.getPoint(path.numPoints() - 1).position);
    }

    public static Command setOdometryFlipped(FlippedPose2d pose, Drive drive) {
        return Commands.runOnce(() -> RobotState.getInstance().setPose(drive.getGyroRotation(), drive.getModulePositions(), pose.getOurs()));
    }

    public static Command followPathFlipped(PathPlannerPath path, Drive drive) {
        return new FollowPathCommand(path, drive::getPose, drive::getRobotMeasuredSpeeds, drive::drivePPVelocity, Drive.autoConfig(), Drive.robotConfig(), AllianceFlipUtil::shouldFlip, drive.translationSubsystem, drive.rotationalSubsystem)
            .deadlineFor(Commands.startEnd(
                () -> Logger.recordOutput("Autonomous/Goal Pose", new Pose2d(getLastPoint(path), path.getGoalEndState().rotation())),
                () -> Logger.recordOutput("Autonomous/Goal Pose", (Pose2d)null)
            ));
    }
    public static Command followPathFlipped(PathPlannerPath path, Drive.Translational drive) {
        return new FollowPathCommand(path, drive.drive::getPose, drive.drive::getRobotMeasuredSpeeds, drive.drive::drivePPVelocity, Drive.autoConfig(), Drive.robotConfig(), AllianceFlipUtil::shouldFlip, drive)
            .deadlineFor(Commands.startEnd(
                () -> Logger.recordOutput("Autonomous/Goal Pose", new Pose2d(getLastPoint(path), path.getGoalEndState().rotation())),
                () -> Logger.recordOutput("Autonomous/Goal Pose", (Pose2d)null)
            ));
    }
    public static Command punch(Intake intake, Puncher puncher) {
        return Commands.parallel(
            intake.eject(),
            puncher.punch()
        )
        .withTimeout(0.25)
        ;
    }
    public static Command intake(Intake intake, Arm arm) {
        return Commands.deadline(
            intake.intake(() -> true),
            arm.floor()
        );
    }
    public static Command stack(Intake intake, Arm arm) {
        return Commands.sequence(
            arm.floor().until(
                arm.atPos
            ),
            Commands.parallel(
                arm.puncher(),
                intake.eject()
            ).until(
                arm.atPos
            )
        );
    }

    public static Command scorePreloadHigh(PathPlannerPath startToScore, Drive drive, Intake intake, Puncher puncher) {
        var goal = AllianceFlipUtil.apply(new Pose2d(getLastPoint(startToScore), startToScore.getGoalEndState().rotation()));
        return Commands.sequence(
            drive.followBluePath(startToScore),
            AutoDrive.preciseToPose(goal, drive).until(
                AutoDrive.withinTolerance(goal, drive)
            ),
            punch(intake, puncher)
        );
    }
    public static Command grabStagedBucket(PathPlannerPath toBucket, Drive drive, Intake intake, Arm arm) {
        return Commands.parallel(
            drive.followBluePath(toBucket),
            arm.puncher().withTimeout(1).andThen(
                intake(intake, arm)
            )
        );
    }
    public static Command scoreStagedBucketHigh(PathPlannerPath toBucket, PathPlannerPath toScore, Drive drive, Intake intake, Arm arm, Puncher puncher) {
        var goal = AllianceFlipUtil.apply(new Pose2d(getLastPoint(toScore), toScore.getGoalEndState().rotation()));
        return Commands.sequence(
            grabStagedBucket(toBucket, drive, intake, arm),
            Commands.sequence(
                drive.followBluePath(toScore),
                    AutoDrive.preciseToPose(goal, drive).until(
                    AutoDrive.withinTolerance(goal, drive)
                ),
                punch(intake, puncher)
            )
            .deadlineFor(arm.puncher())
        );
    }
    public static Command scoreStagedBucketStacking(PathPlannerPath toBucket, PathPlannerPath toScore, Drive drive, Intake intake, Arm arm, Puncher puncher) {
        return Commands.sequence(
            grabStagedBucket(toBucket, drive, intake, arm),
            drive.followBluePath(toScore).deadlineFor(arm.puncher()),
            stack(intake, arm)
        );
    }

    public static class AutoPaths {
        private static final Map<String, PathPlannerPath> loadedPaths = new HashMap<>();
        private static boolean preloading;
        public static void preload() {
            preloading = true;
            // load paths
            loadChoreoTrajectory("Inner To Stacking");
            loadChoreoTrajectory("Inner To Source");
            loadChoreoTrajectory("Inner To Stacking");
            loadChoreoTrajectory("Outer To Stacking");
            loadChoreoTrajectory("Stacking To Bucket4");
            loadChoreoTrajectory("Stacking To Bucket3");
            loadChoreoTrajectory("Source To Bucket2");
            loadChoreoTrajectory("Source To Bucket3");
            loadChoreoTrajectory("Source To Bucket4");
            loadChoreoTrajectory("Stacking To Bucket2");
            loadChoreoTrajectory("Bucket3 To Source");
            loadChoreoTrajectory("Bucket4 To Source");
            loadChoreoTrajectory("Bucket4 To Stacking");
            loadChoreoTrajectory("Bucket3 To Stacking");
            loadChoreoTrajectory("Bucket2 To Stacking");
            loadChoreoTrajectory("Stacking0 To Bucket2");
            loadChoreoTrajectory("Bucket2 To Stacking1");
            loadChoreoTrajectory("Stacking1 To Bucket1");
            preloading = false;
            System.out.println("[Init AutoPaths] Loaded paths");
            PathPlannerLogging.setLogActivePathCallback((path) -> Logger.recordOutput("Autonomous/Path", path.toArray(Pose2d[]::new)));
            PathPlannerLogging.setLogTargetPoseCallback((target) -> Logger.recordOutput("Autonomous/Target Pose", target));
        }

        public static PathPlannerPath loadPath(String name) {
            if(loadedPaths.containsKey(name)) {
                return loadedPaths.get(name);
            } else {
                if(!preloading) new Alert("[AutoPaths] Loading \"" + name + "\" which wasn't preloaded. Please add path to AutoPaths.preload()", AlertType.kWarning).set(true);
                try {
                    var path = PathPlannerPath.fromPathFile(name);
                    loadedPaths.put(name, path);
                    return path;
                } catch (Exception e) {
                    return null;
                }
            }
        }

        public static PathPlannerPath loadChoreoTrajectory(String name) {
            if(loadedPaths.containsKey(name)) {
                return loadedPaths.get(name);
            } else {
                if(!preloading) new Alert("[AutoPaths] Loading \"" + name + "\" which wasn't preloaded. Please add path to AutoPaths.preload()", AlertType.kWarning).set(true);
                try {
                    var path = PathPlannerPath.fromChoreoTrajectory(name);
                    loadedPaths.put(name, path);
                    return path;
                } catch (Exception e) {
                    return null;
                }
            }
        }

        public static String getName(PathPlannerPath path) {
            return loadedPaths.entrySet().stream().filter((e) -> e.getValue() == path).map((e) -> e.getKey()).findAny().orElse("Unknown Path");
        }
    }
}