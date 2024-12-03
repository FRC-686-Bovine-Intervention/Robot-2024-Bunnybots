package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.constants.FieldConstants;
import frc.util.PoseBoundingBoxUtil.BoundingBox;
import frc.util.robotStructure.CameraMount;

public final class VisionConstants {
    public static final Distance zMargin = Meters.of(0.75);
    public static final BoundingBox acceptableFieldBox = BoundingBox.rectangle(
        new Translation2d(

        ),
        new Translation2d(
            FieldConstants.fieldLength,
            FieldConstants.fieldWidth
        )
    );

    public static class CameraConstants {
        public final String hardwareName;
        public final CameraMount mount;

        public CameraConstants(String hardwareName, CameraMount mount) {
            this.hardwareName = hardwareName;
            this.mount = mount;
        }
    }
    public static class ApriltagCameraConstants extends CameraConstants {
        public final double cameraStdCoef;
        public final Distance trustDistance;

        public ApriltagCameraConstants(String hardwareName, CameraMount mount, double cameraStdCoef, Distance trustDistance) {
            super(hardwareName, mount);
            this.cameraStdCoef = cameraStdCoef;
            this.trustDistance = trustDistance;
        }
    }

    public static final ApriltagCameraConstants frontLeftApriltagCamera = new ApriltagCameraConstants(
        "Front Left",
        new CameraMount(new Transform3d(
            new Translation3d(
                Meters.of(+0.220594),
                Meters.of(+0.280635),
                Meters.of(+0.234983)
            ),
            new Rotation3d(
                Degrees.of(+0),
                Degrees.of(+0),
                Degrees.of(+135)
            )
            .rotateBy(new Rotation3d(
                Degrees.of(+0),
                Degrees.of(+20),
                Degrees.of(+0)
            ))
        )),
        1.0,
        Meters.of(8)
    );
    public static final ApriltagCameraConstants frontRightApriltagCamera = new ApriltagCameraConstants(
        "Front Right",
        new CameraMount(new Transform3d(
            new Translation3d(
                Meters.of(+0.220594),
                Meters.of(-0.280635),
                Meters.of(+0.234983)
            ),
            new Rotation3d(
                Degrees.of(+0),
                Degrees.of(+0),
                Degrees.of(-135)
            )
            .rotateBy(new Rotation3d(
                Degrees.of(+0),
                Degrees.of(+20),
                Degrees.of(+0)
            ))
        )),
        1.0,
        Meters.of(8)
    );
    public static final ApriltagCameraConstants backLeftApriltagCamera = new ApriltagCameraConstants(
        "Back Left",
        new CameraMount(new Transform3d(
            new Translation3d(
                Meters.of(-0.208321),
                Meters.of(+0.254315),
                Meters.of(+0.234983)
            ),
            new Rotation3d(
                Degrees.of(+0),
                Degrees.of(+0),
                Degrees.of(+5)
            )
            .rotateBy(new Rotation3d(
                Degrees.of(+0),
                Degrees.of(-20),
                Degrees.of(+0)
            ))
        )),
        1.0,
        Meters.of(8)
    );
    public static final ApriltagCameraConstants backRightApriltagCamera = new ApriltagCameraConstants(
        "Back Right",
        new CameraMount(new Transform3d(
            new Translation3d(
                Meters.of(-0.209274),
                Meters.of(-0.255268),
                Meters.of(+0.234983)
            ),
            new Rotation3d(
                Degrees.of(+0),
                Degrees.of(+0),
                Degrees.of(-5)
            )
            .rotateBy(new Rotation3d(
                Degrees.of(+0),
                Degrees.of(-20),
                Degrees.of(+0)
            ))
        )),
        1.0,
        Meters.of(8)
    );

    // TODO: figure out vision stdDevs
    public static final double singleTagAmbiguityCutoff = 0.05;
    public static final double minimumStdDev = 0.5;
    public static final double stdDevEulerMultiplier = 0.3;
    public static final double stdDevDistanceMultiplier = 0.4;
}
