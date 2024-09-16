package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Wraps SwerveDrivePoseEstimator and adds Logging and Field2d */

public class RobotState {
    private static RobotState instance;

    public static RobotState getInstance() {
        if (instance == null)
            instance = new RobotState();
        return instance;
    }

    private SwerveDrivePoseEstimator poseEstimator;
    private Field2d field = new Field2d();

    public void initializePoseEstimator(
            SwerveDriveKinematics kinematics,
            Rotation2d gyroAngle,
            SwerveModulePosition[] modulePositions,
            Pose2d initialPoseMeters) {
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, gyroAngle, modulePositions, initialPoseMeters);
        SmartDashboard.putData(field);
        logOdometry();
    }

    public void addDriveMeasurement(Rotation2d rotation, SwerveModulePosition[] modulePositions) {
        poseEstimator.update(rotation, modulePositions);
    }

    public void addVisionMeasurement(Pose2d pose, Matrix<N3, N1> stdDevs, double timestamp) {
        poseEstimator.addVisionMeasurement(pose, timestamp, stdDevs);
        field.setRobotPose(getPose());
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setPose(Rotation2d rotation, SwerveModulePosition[] modulePositions, Pose2d fieldToVehicle) {
        poseEstimator.resetPosition(rotation, modulePositions, fieldToVehicle);
    }

    public void logOdometry() {
        Pose2d pose = getPose();
        Logger.recordOutput("Odometry/Robot", pose);
        field.setRobotPose(pose);
    }

}
