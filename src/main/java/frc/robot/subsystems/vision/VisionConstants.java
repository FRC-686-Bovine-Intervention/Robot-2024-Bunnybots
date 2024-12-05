package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.util.robotStructure.CameraMount;

public final class VisionConstants {
    public static class CameraConstants {
        public final String hardwareName;
        public final CameraMount mount;

        public CameraConstants(String hardwareName, CameraMount mount) {
            this.hardwareName = hardwareName;
            this.mount = mount;
        }
    }

    public static final CameraMount frontLeftModuleMount = new CameraMount(new Transform3d(
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
            Degrees.of(+0),
            Degrees.of(+0)
        ))
    ));
    public static final CameraMount frontRightModuleMount = new CameraMount(new Transform3d(
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
            Degrees.of(+0),
            Degrees.of(+0)
        ))
    ));
    public static final CameraMount backLeftModuleMount = new CameraMount(new Transform3d(
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
    ));
    public static final CameraMount backRightModuleMount = new CameraMount(new Transform3d(
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
    ));
    public static final CameraMount flagStickMount = new CameraMount(new Transform3d(
        new Translation3d(
            Meters.of(-0.219548),
            Meters.of(-0.195904),
            Meters.of(+1.147068)
        ),
        new Rotation3d(
            Degrees.of(+0),
            Degrees.of(+30),
            Degrees.of(+0)
        )
        .rotateBy(new Rotation3d(
            Degrees.of(+0),
            Degrees.of(+0),
            Degrees.of(+10)
        ))
    ));
}
