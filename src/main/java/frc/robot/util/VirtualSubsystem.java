// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Watchdog;
import frc.robot.Robot;

/**
 * Represents a subsystem unit that requires a periodic callback but not require a hardware mutex.
 */
public abstract class VirtualSubsystem {
  private static List<VirtualSubsystem> subsystems = new ArrayList<>();
  private static final Watchdog watchdog = new Watchdog(Robot.defaultPeriodSecs, () -> {});

  public VirtualSubsystem() {
    subsystems.add(this);
  }

  /** Calls {@link #periodic()} on all virtual subsystems. */
  public static void periodicAll() {
    watchdog.reset();
    for (var subsystem : subsystems) {
      subsystem.periodic();
      watchdog.addEpoch(subsystem.getClass().getSimpleName());
    }
    if (watchdog.isExpired()) {
      System.out.println("VirtualSubsystem loop overrun");
      watchdog.printEpochs();
    }
  }

  /** This method is called periodically once per loop cycle. */
  public abstract void periodic();
}
