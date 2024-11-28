package frc.robot.auto;

import java.util.HashMap;
import java.util.Map;

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
import frc.robot.subsystems.drive.Drive;
import frc.util.AllianceFlipUtil;
import frc.util.AllianceFlipUtil.FlippedPose2d;

public class AutoCommons {
    public static enum CenterNote {
        Note1,
        Note2,
        Note3,
        Note4,
        Note5,
        ;
        public Map.Entry<String, CenterNote> toEntry() {
            return Map.entry(this.name(), this);
        }
    }

    public static Command setOdometryFlipped(FlippedPose2d pose, Drive drive) {
        return Commands.runOnce(() -> RobotState.getInstance().setPose(drive.getGyroRotation(), drive.getModulePositions(), pose.getOurs()));
    }

    public static Command followPathFlipped(PathPlannerPath path, Drive drive) {
        return new FollowPathCommand(path, drive::getPose, drive::getRobotMeasuredSpeeds, drive::drivePPVelocity, Drive.autoConfig(), Drive.robotConfig(), AllianceFlipUtil::shouldFlip, drive.translationSubsystem, drive.rotationalSubsystem)
            .deadlineFor(Commands.startEnd(
                () -> Logger.recordOutput("Autonomous/Goal Pose", new Pose2d(getLastPoint(path), path.getGoalEndState().rotation())),
                () -> Logger.recordOutput("Autonomous/Goal Pose", (Pose2d)null)
            ))
        ;
    }
    public static Command followPathFlipped(PathPlannerPath path, Drive.Translational drive) {
        return new FollowPathCommand(path, drive.drive::getPose, drive.drive::getRobotMeasuredSpeeds, drive.drive::drivePPVelocity, Drive.autoConfig(), Drive.robotConfig(), AllianceFlipUtil::shouldFlip, drive)
            .deadlineFor(Commands.startEnd(
                () -> Logger.recordOutput("Autonomous/Goal Pose", new Pose2d(getLastPoint(path), path.getGoalEndState().rotation())),
                () -> Logger.recordOutput("Autonomous/Goal Pose", (Pose2d)null)
            ))
        ;
    }

    public static Translation2d getFirstPoint(PathPlannerPath path) {
        return AllianceFlipUtil.apply(path.getPoint(0).position);
    }

    public static Translation2d getLastPoint(PathPlannerPath path) {
        return AllianceFlipUtil.apply(path.getPoint(path.numPoints() - 1).position);
    }

    public static class AutoPaths {
        private static final Map<String, PathPlannerPath> loadedPaths = new HashMap<>();
        private static boolean preloading;
        public static void preload() {
            preloading = true;
            // loadPath("Amp Start to Spike");
            preloading = false;
            System.out.println("[Init AutoPaths] Loaded paths");
            PathPlannerLogging.setLogActivePathCallback((path) -> Logger.recordOutput("Autonomous/Path", path.toArray(Pose2d[]::new)));
            PathPlannerLogging.setLogTargetPoseCallback((target) -> Logger.recordOutput("Autonomous/Target Pose", target));
        }

        public static PathPlannerPath loadPath(String name) {
            return loadPath(name, false);
        }

        public static PathPlannerPath loadPath(String name, boolean isStagePath) {
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

        public static String getName(PathPlannerPath path) {
            return loadedPaths.entrySet().stream().filter((e) -> e.getValue() == path).map((e) -> e.getKey()).findAny().orElse("Unknown Path");
        }
    }
}
