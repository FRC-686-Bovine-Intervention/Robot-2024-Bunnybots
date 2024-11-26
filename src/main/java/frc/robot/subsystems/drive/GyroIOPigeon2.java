// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import frc.robot.constants.HardwareDevices;
import frc.robot.constants.RobotConstants;

/** IO implementation for Pigeon2 */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon = HardwareDevices.pigeonID.pigeon2();

  public GyroIOPigeon2() {
    var config = new Pigeon2Configuration();
    // change factory defaults here
    config.MountPose.MountPoseYaw = -179.67636108398438;    // pigeon2 oriented with x forward, y left, z up
    config.MountPose.MountPosePitch = -0.25307801365852356;
    config.MountPose.MountPoseRoll = -0.5043123960494995;
    pigeon.getConfigurator().apply(config);

    // set signals to an appropriate rate
    pigeon.getYaw().setUpdateFrequency(RobotConstants.rioUpdateFrequency);

    pigeon.setYaw(0);
  }

  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = pigeon.getYaw().getStatus().isOK();

    inputs.rotation = pigeon.getRotation3d();

    inputs.yawVelocity = pigeon.getAngularVelocityZWorld().getValue();   // ccw+
    inputs.pitchVelocity = pigeon.getAngularVelocityYWorld().getValue().unaryMinus();   // up+
    inputs.rollVelocity = pigeon.getAngularVelocityXWorld().getValue().unaryMinus();   // ccw+
  }
}
