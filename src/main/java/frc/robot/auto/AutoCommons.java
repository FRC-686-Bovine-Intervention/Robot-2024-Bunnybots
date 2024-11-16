package frc.robot.auto;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
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
        Far(FlippedPose2d.fromBlue(
            new Pose2d(
                new Translation2d(1.218, 0.883), Rotation2d.fromDegrees(0.0)
            ))),
        Middle(FlippedPose2d.fromBlue(
            new Pose2d(
                new Translation2d(1.260, 2.528), Rotation2d.fromDegrees(0.0)
            ))),
        Close(FlippedPose2d.fromBlue(
            new Pose2d(
                new Translation2d(1.260, 5.127), Rotation2d.fromDegrees(0.0)
            ))),
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

    public static enum Canister implements ToEntry<Canister> {
        Canister1(1),
        Canister2(2),
        Canister3(3),
        Canister4(4)
        ;
        private int number;
        Canister(int number) {
            this.number = number;
        }
        @Override
        public Entry<String, Canister> toEntry() {
            return Map.entry(Integer.toString(number), this);
        }
        @Override
        public String toString() {
            return Integer.toString(this.number);
        }
    }

    public static Translation2d getFirstPoint(PathPlannerPath path) {
        PathType.Far.toEntry();
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

    public static class AutoPaths {
        private static final Map<String, PathPlannerPath> loadedPaths = new HashMap<>();
        private static boolean preloading;
        public static void preload() {
            preloading = true;
            // load paths
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
                if(!preloading) new Alert("[AutoPaths] Loading \"" + name + "\" which wasn't preloaded. Please add path to AutoPaths.preload()", AlertType.WARNING).set(true);
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