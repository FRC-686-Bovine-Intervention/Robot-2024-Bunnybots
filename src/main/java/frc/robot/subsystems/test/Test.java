package frc.robot.subsystems.test;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.Cooldown;
import frc.util.robotStructure.CameraMount;
import frc.util.robotStructure.GamepiecePose;
import frc.util.robotStructure.Mechanism3d;
import frc.util.robotStructure.angle.ArmMech;
import frc.util.robotStructure.linear.ExtenderMech;

public class Test extends SubsystemBase {
    
    public final ArmMech armMech = new ArmMech(
        new Transform3d(
            new Translation3d(
                Inches.of(5.625000),
                Inches.of(0),
                Inches.of(14.250000)
            ),
            Rotation3d.kZero
        )
    );
    public final GamepiecePose uprightGamepiecePose = new GamepiecePose(
        new Transform3d(
            new Translation3d(
                Inches.of(20.750000),
                Inches.of(0),
                Inches.of(0.250000).minus(Inches.of(14.500000))
            ),
            Rotation3d.kZero
        )
    );
    public final GamepiecePose invertedGamepiecePose = new GamepiecePose(
        new Transform3d(
            new Translation3d(
                Inches.of(20.750000),
                Inches.of(0),
                Inches.of(0.250000)
            ),
            new Rotation3d(Math.PI, 0, 0)
        )
    );

    private final BooleanSupplier increaseArm;
    private final BooleanSupplier decreaseArm;
    public Test(BooleanSupplier increaseArm, BooleanSupplier decreaseArm) {
        this.increaseArm = increaseArm;
        this.decreaseArm = decreaseArm;
    }

    private final Cooldown cooldown = new Cooldown(0.25);
    private final MutAngle angle = Radians.mutable(0);

    @Override
    public void periodic() {
        if (cooldown.hasExpired()) {
            if (increaseArm.getAsBoolean()) {
                angle.mut_acc(Degrees.of(15));
                cooldown.reset();
            }
            if (decreaseArm.getAsBoolean()) {
                angle.mut_minus(Degrees.of(15));
                cooldown.reset();
            }
        }

        armMech.setData(angle);
        armMech.log(Mechanism3d.KEY + "/Arm");
        
        Logger.recordOutput("Field/Arm", armMech.getFieldRelative());
        Logger.recordOutput("Field/Upright Bucket", uprightGamepiecePose.getFieldRelative());
        Logger.recordOutput("Field/Inverted Bucket", invertedGamepiecePose.getFieldRelative());
        CameraMount.logOverrides();
    }
}
