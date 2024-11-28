// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.AngularVelocity;

public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public Rotation3d rotation = new Rotation3d();
    public AngularVelocity yawVelocity = RadiansPerSecond.zero();
    public AngularVelocity pitchVelocity = RadiansPerSecond.zero();
    public AngularVelocity rollVelocity = RadiansPerSecond.zero();
  }

  public default void updateInputs(GyroIOInputs inputs) {}
  public default void resetYaw(Measure<AngleUnit> yaw) {}
}
