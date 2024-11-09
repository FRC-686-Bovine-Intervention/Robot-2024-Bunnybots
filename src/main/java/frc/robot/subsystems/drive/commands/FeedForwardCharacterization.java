// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive.commands;

import java.util.LinkedList;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.util.PolynomialRegression;

public class FeedForwardCharacterization extends Command {
  private static final double startDelaySecs = 2.0;
  private static final double rampRateVoltsPerSec = 0.1;

  private final boolean forwards;
  private final boolean isDrive;

  private final FeedForwardCharacterizationData dataPrimary;
  private final FeedForwardCharacterizationData dataSecondary;
  private final Consumer<Double> voltageConsumerSimple;
  private final BiConsumer<Double, Double> voltageConsumerDrive;
  private final DoubleSupplier velocitySupplierPrimary;
  private final DoubleSupplier velocitySupplierSecondary;

  private final Timer timer = new Timer();

  /** Creates a new FeedForwardCharacterization for a differential drive. */
  public FeedForwardCharacterization(
      Subsystem drive,
      boolean forwards,
      FeedForwardCharacterizationData leftData,
      FeedForwardCharacterizationData rightData,
      BiConsumer<Double, Double> voltageConsumer,
      DoubleSupplier leftVelocitySupplier,
      DoubleSupplier rightVelocitySupplier) {
    addRequirements(drive);
    this.forwards = forwards;
    this.isDrive = true;
    this.dataPrimary = leftData;
    this.dataSecondary = rightData;
    this.voltageConsumerSimple = null;
    this.voltageConsumerDrive = voltageConsumer;
    this.velocitySupplierPrimary = leftVelocitySupplier;
    this.velocitySupplierSecondary = rightVelocitySupplier;
  }

  /** Creates a new FeedForwardCharacterization for a simple subsystem. */
  public FeedForwardCharacterization(
      Subsystem subsystem,
      boolean forwards,
      FeedForwardCharacterizationData data,
      Consumer<Double> voltageConsumer,
      DoubleSupplier velocitySupplier) {
    addRequirements(subsystem);
    this.forwards = forwards;
    this.isDrive = false;
    this.dataPrimary = data;
    this.dataSecondary = null;
    this.voltageConsumerSimple = voltageConsumer;
    this.voltageConsumerDrive = null;
    this.velocitySupplierPrimary = velocitySupplier;
    this.velocitySupplierSecondary = null;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() < startDelaySecs) {
      if (isDrive) {
        voltageConsumerDrive.accept(0.0, 0.0);
      } else {
        voltageConsumerSimple.accept(0.0);
      }
    } else {
      double voltage = (timer.get() - startDelaySecs) * rampRateVoltsPerSec * (forwards ? 1 : -1);
      if (isDrive) {
        voltageConsumerDrive.accept(voltage, voltage);
      } else {
        voltageConsumerSimple.accept(voltage);
      }
      dataPrimary.add(velocitySupplierPrimary.getAsDouble(), voltage);
      if (isDrive) {
        dataSecondary.add(velocitySupplierSecondary.getAsDouble(), voltage);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (isDrive) {
      voltageConsumerDrive.accept(0.0, 0.0);
    } else {
      voltageConsumerSimple.accept(0.0);
    }
    timer.stop();
    dataPrimary.print();
    if (isDrive) {
      dataSecondary.print();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public static class FeedForwardCharacterizationData {
    private final String name;
    private final List<Double> velocityData = new LinkedList<>();
    private final List<Double> voltageData = new LinkedList<>();

    public FeedForwardCharacterizationData(String name) {
      this.name = name;
    }

    public void add(double velocity, double voltage) {
      if (Math.abs(velocity) > 1E-4) {
        velocityData.add(Math.abs(velocity));
        voltageData.add(Math.abs(voltage));
      }
    }

    public void print() {
      if (velocityData.size() == 0 || voltageData.size() == 0) {
        return;
      }

      PolynomialRegression regression =
          new PolynomialRegression(
              velocityData.stream().mapToDouble(Double::doubleValue).toArray(),
              voltageData.stream().mapToDouble(Double::doubleValue).toArray(),
              1);

      System.out.println("FF Characterization Results (" + name + "):");
      System.out.println("\tCount=" + Integer.toString(velocityData.size()) + "");
      System.out.println(String.format("\tR2=%.5f", regression.R2()));
      System.out.println(String.format("\tkS=%.5f", regression.beta(0)));
      System.out.println(String.format("\tkV=%.5f", regression.beta(1)));
    }
  }
}
