package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;

import java.util.Arrays;
import java.util.function.Function;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.function.BooleanConsumer;
import frc.robot.RobotState;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.vision.apriltag.ApriltagCamera;
import frc.robot.subsystems.vision.apriltag.ApriltagCameraIO;

public final class VisionConstants {
    public static enum Camera {
        LeftApriltag(
            "Left Apriltag Cam",
            0.6,
            5,
            new Transform3d(
                new Translation3d(
                    Inches.of(+12.225),
                    Inches.of(+9.557),
                    Inches.of(+10.932)
                ),
                new Rotation3d(
                    Units.degreesToRadians(0),
                    Units.degreesToRadians(-12.348-5),
                    Units.degreesToRadians(+0)
                )
            ),
            (connected) -> {
                Leds.getInstance().lAprilConnected = connected;
            }
            // robotToCameraFromCalibTag(
            //     new Transform3d(
            //         new Translation3d(
            //             4.62,
            //             0,
            //             1.465
            //         ),
            //         new Rotation3d(
            //             0,0,Math.PI
            //         )
            //     ),
            //     new Transform3d(
            //         new Translation3d(
            //             4.26,
            //             -0.126,
            //             1.145
            //         ),
            //         new Rotation3d(
            //             new Quaternion(
            //                 0.033,
            //                 0.03,
            //                 0,
            //                 1
            //             )
            //         )
            //     )
            // )
        ),
        RightApriltag(
            "Right Apriltag Cam",
            1,
            4,
            new Transform3d(
                new Translation3d(
                    Inches.of(+12.225),
                    Inches.of(-9.557),
                    Inches.of(+10.932)
                ),
                new Rotation3d(
                    Units.degreesToRadians(0),
                    Units.degreesToRadians(-32.414),
                    Units.degreesToRadians(0)
                )
                // new Rotation3d(
                //     Units.degreesToRadians(-(90.0-87.654)),
                //     Units.degreesToRadians(-32.414),
                //     Units.degreesToRadians(-9.707)
                // )
            ),
            (connected) -> {
                Leds.getInstance().rAprilConnected = connected;
            }
        ),
        NoteVision(
            "Note Cam",
            0,
            0,
            new Transform3d(
                new Translation3d(
                    Inches.of(-14.047),
                    Inches.of(+0),
                    Inches.of(+12.584)
                ),
                new Rotation3d(
                    0,
                    Degrees.of(15).in(Radians),
                    Math.PI
                )
            ),
            (connected) -> {
                Leds.getInstance().nVisionConnected = connected;
            }
        ),
        ;
        public final String hardwareName;
        private final Transform3d intermediateToCamera;
        public final double cameraStdCoef;
        public final double trustDistance;
        public final BooleanConsumer connectedConsumer;
        private Supplier<Transform3d> robotToIntermediate;
        Camera(String hardwareName, double cameraStdCoef, double trustDistance, Transform3d finalToCamera, BooleanConsumer connectedConsumer) {
            this.hardwareName = hardwareName;
            this.cameraStdCoef = cameraStdCoef;
            this.trustDistance = trustDistance;
            this.intermediateToCamera = finalToCamera;
            this.connectedConsumer = connectedConsumer;
            this.robotToIntermediate = Transform3d::new;
        }
        @SuppressWarnings("unused")
        private static Transform3d robotToCameraFromCalibTag(Transform3d robotToCalibTag, Transform3d cameraToCalibTag) {
            return robotToCalibTag.plus(cameraToCalibTag.inverse());
        }
        public Camera withRobotToIntermediate(Supplier<Transform3d> robotToFinal) {
            this.robotToIntermediate = robotToFinal;
            return this;
        }

        public Transform3d getRobotToCam() {
            return robotToIntermediate.get().plus(intermediateToCamera);
        }

        public ApriltagCamera toApriltagCamera() {
            return new ApriltagCamera(this, new ApriltagCameraIO(){});
        }
        public ApriltagCamera toApriltagCamera(Function<Camera, ? extends ApriltagCameraIO> function) {
            return new ApriltagCamera(this, function.apply(this));
        }

        public static void logCameraOverrides() {
            Logger.recordOutput("Camera Overrides", 
                Arrays.stream(Camera.values())
                .map(
                    (cam) -> 
                        new Transform3d(
                            new Pose3d(),
                            new Pose3d(RobotState.getInstance().getPose())
                        )
                        .plus(cam.getRobotToCam())
                )
                .toArray(Transform3d[]::new)
            );
        }
    }

    // TODO: figure out vision stdDevs
    public static final double singleTagAmbiguityCutoff = 0.05;
    public static final double minimumStdDev = 0.5;
    public static final double stdDevEulerMultiplier = 0.3;
    public static final double stdDevDistanceMultiplier = 0.4;
}
