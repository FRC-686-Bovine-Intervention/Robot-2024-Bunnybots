// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.LoggedTunableMeasure;
import frc.util.SuppliedEdgeDetector;

public class Arm extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  public static final LoggedTunableMeasure<AngleUnit> setPoint1 = new LoggedTunableMeasure<>("Arm/Angles/Set Point 1", Degrees.of(0));
  public static final LoggedTunableMeasure<AngleUnit> setPoint2 = new LoggedTunableMeasure<>("Arm/Angles/Set Point 2", Degrees.of(90));
  
  private final SuppliedEdgeDetector increaseEdgeDetector;
  private final SuppliedEdgeDetector decreaseEdgeDetector;

  private Measure<AngleUnit> runtimeOffset = Degrees.of(0);

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
  }
}
