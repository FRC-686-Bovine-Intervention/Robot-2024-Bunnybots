package frc.robot.subsystems.vision.apriltag;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionConstants.CameraConstants;
import frc.util.PoseBoundingBoxUtil.BoundingBox;
import frc.util.robotStructure.CameraMount;

public class ApriltagVisionConstants {
    public static final Distance zMargin = Inches.of(6);
    public static final BoundingBox acceptableFieldBox = BoundingBox.rectangle(
        new Translation2d(

        ),
        new Translation2d(
            FieldConstants.fieldLength,
            FieldConstants.fieldWidth
        )
    );

    public static class ApriltagCameraConstants extends CameraConstants {
        public final double cameraStdCoef;
        public final Distance trustDistance;

        public ApriltagCameraConstants(
            String hardwareName,
            CameraMount mount,
            SimCameraProperties simCamProps,
            double cameraStdCoef,
            Distance trustDistance
        ) {
            super(hardwareName, mount, simCamProps);
            this.cameraStdCoef = cameraStdCoef;
            this.trustDistance = trustDistance;
        }
    }

    public static final ApriltagCameraConstants frontLeftApriltagCamera = new ApriltagCameraConstants(
        "Front Left",
        VisionConstants.frontLeftModuleMount,
        VisionConstants.apriltagSimCamProps,
        1.0,
        Meters.of(8)
    );
    public static final ApriltagCameraConstants frontRightApriltagCamera = new ApriltagCameraConstants(
        "Front Right",
        VisionConstants.frontRightModuleMount,
        VisionConstants.apriltagSimCamProps,
        1.0,
        Meters.of(8)
    );
    public static final ApriltagCameraConstants backLeftApriltagCamera = new ApriltagCameraConstants(
        "Back Left",
        VisionConstants.backLeftModuleMount,
        VisionConstants.apriltagSimCamProps,
        1.0,
        Meters.of(8)
    );
    public static final ApriltagCameraConstants backRightApriltagCamera = new ApriltagCameraConstants(
        "Back Right",
        VisionConstants.backRightModuleMount,
        VisionConstants.apriltagSimCamProps,
        1.0,
        Meters.of(8)
    );

    // TODO: figure out vision stdDevs
    // public static final double singleTagAmbiguityCutoff = 0.05;
    // public static final double minimumStdDev = 0.5;
    // public static final double stdDevEulerMultiplier = 0.3;
    // public static final double stdDevDistanceMultiplier = 0.4;
}
