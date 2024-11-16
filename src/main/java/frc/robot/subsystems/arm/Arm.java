// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.Cooldown;
import frc.util.LoggedInternalButton;
import frc.util.LoggedTunableMeasure;
import frc.util.SuppliedEdgeDetector;
import frc.util.robotStructure.GamepiecePose;
import frc.util.robotStructure.Mechanism3d;
import frc.util.robotStructure.angle.ArmMech;

public class Arm extends SubsystemBase {
    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    public final ArmMech mech = new ArmMech(
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

    public static final LoggedTunableMeasure<AngleUnit> setPoint1 = new LoggedTunableMeasure<>("Arm/Angles/Set Point 1", Degrees.of(0));
    public static final LoggedTunableMeasure<AngleUnit> setPoint2 = new LoggedTunableMeasure<>("Arm/Angles/Set Point 2", Degrees.of(90));
    
    private final SuppliedEdgeDetector increaseEdgeDetector;
    private final SuppliedEdgeDetector decreaseEdgeDetector;

    private final MutAngle offset = Degrees.mutable(0);
    private final Cooldown cooldown = new Cooldown(0.25);
    private final LoggedTunableMeasure<AngleUnit> offsetAdjustment = new LoggedTunableMeasure<>("Arm/Offset Adjustment", Degrees.of(0));
    private final LoggedTunableMeasure<AngleUnit> variablePosAdjustment = new LoggedTunableMeasure<>("Arm/Variable Position Adjustment", Degrees.of(0));

    public static final LoggedTunableMeasure<AngleUnit> tolerance = new LoggedTunableMeasure<>("Arm/Tolerance", Degrees.of(2));
    public final LoggedInternalButton atPos = new LoggedInternalButton("Arm/At Position");

    public Arm(ArmIO io, BooleanSupplier increaseRuntimeOffset, BooleanSupplier decreaseRuntimeOffset) {
        System.out.println("[Init Arm] Instantiating Arm");
        this.io = io;
        System.out.println("[Init Arm] Arm IO: " + this.io.getClass().getSimpleName());
        SmartDashboard.putData("Subsystems/Arm", this);
        this.increaseEdgeDetector = new SuppliedEdgeDetector(increaseRuntimeOffset);
        this.decreaseEdgeDetector = new SuppliedEdgeDetector(decreaseRuntimeOffset);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Arm", inputs);
        if (cooldown.hasExpired()) {
            if (increaseEdgeDetector.risingEdge()) {
                cooldown.reset();
                offset.mut_acc(offsetAdjustment.in(Degrees));
            }
            if(decreaseEdgeDetector.risingEdge()) {
                cooldown.reset();
                offset.mut_acc(-offsetAdjustment.in(Degrees));
            }
        }

        if(increaseEdgeDetector.risingEdge() || decreaseEdgeDetector.risingEdge()) {
            io.setOffset(offset);
        }
        
        mech.log(Mechanism3d.KEY + "/Arm");
    }

    public Command genCommand(String name, Supplier<Measure<AngleUnit>> posSupplier) {
        var subsystem = this;
        return new Command() {
            {
                setName(name);
                addRequirements(subsystem);
            }

            @Override
            public void initialize() {
                execute();
            }

            @Override
            public void execute() {
                atPos.setPressed(MathUtil.isNear(
                    posSupplier.get().in(Degrees),
                    inputs.encoder.position.in(Degrees),
                    tolerance.in(Degrees)
                ));
                io.setPos(posSupplier.get()); 
            }

            @Override
            public void end(boolean interrupted) {
                io.stop();
            }
        };
    }

    public Command customVariable(BooleanSupplier increase, BooleanSupplier decrease) {
        return genCommand("Custom", new Supplier<Measure<AngleUnit>>() {
            private final MutAngle pos = Degrees.mutable(0);
            private final Cooldown cooldown = new Cooldown(0.125);
            public Measure<AngleUnit> get() {
                    Logger.recordOutput("Custom Shoot/Pivot Angle", pos);
                    if(!cooldown.hasExpired()) {
                        return pos;
                    }
                    if(increase.getAsBoolean()) {
                        cooldown.reset();
                        pos.mut_acc(variablePosAdjustment.in(Degrees));
                    }
                    if(decrease.getAsBoolean() && pos.gt(Degrees.zero())) {
                        cooldown.reset();
                        pos.mut_acc(-variablePosAdjustment.in(Degrees));
                    }

                    return pos;
                }
        });
    }

    public Command coast() {
        var subsystem = this;
        return new Command() {
            {
                setName("Coast");
                addRequirements(subsystem);
            }

            @Override
            public void initialize() {
                io.setCoast(true);
            }

            @Override
            public void end(boolean interrupted) {
                io.setCoast(false);
            }

            @Override
            public boolean runsWhenDisabled() {
                return true;
            }
        };
    }
}
