package frc.robot.constants;

import frc.util.hardwareID.CANBus;
import frc.util.hardwareID.CANDevice;
import frc.util.hardwareID.PnuematicsControl;
import frc.util.hardwareID.PnuematicsControl.SolenoidPort;
import frc.util.hardwareID.rioPorts.DIOPort;
import frc.util.hardwareID.rioPorts.PWMPort;

public class HardwareDevices {
    /*
     * PDH Ports
     * 10:                      9:
     * 11:                      8:
     * 12:                      7:
     * 13:                      6:
     * 14: PCM                  5: Intake Motor
     * 15: MPM                  4: Arm Motor
     * 16: Front Right Turn     3: Front Left Turn
     * 17: Front Right Drive    2: Front Left Drive
     * 18: Back Right Turn      1: Back Left Turn
     * 19: Back Right Drive     0: Back Left Drive
     * 20: RoboRIO
     * 21: Radio DC
     * 22: Radio POE
     */
    public static final CANBus rio = CANBus.rio();
    public static final CANBus canivore = CANBus.newBus("canivore");

    public static final CANDevice pnuematicsHubID = rio.id(1);
    public static final PnuematicsControl pnuematicsHub = pnuematicsHubID.pneumaticHub();
    
    // Drive
    public static final CANDevice pigeonID = canivore.id(0);
    // | Front Left
    public static final CANDevice frontLeftDriveMotorID = canivore.id(1);
    public static final CANDevice frontLeftTurnMotorID = rio.id(1);
    // | Front Right
    public static final CANDevice frontRightDriveMotorID = canivore.id(2);
    public static final CANDevice frontRightTurnMotorID = rio.id(2);
    // | Back Left
    public static final CANDevice backLeftDriveMotorID = canivore.id(3);
    public static final CANDevice backLeftTurnMotorID = rio.id(3);
    // | Back Right
    public static final CANDevice backRightDriveMotorID = canivore.id(4);
    public static final CANDevice backRightTurnMotorID = rio.id(4);

    // Arm
    public static final CANDevice armMotorID = canivore.id(5);
    public static final CANDevice armEncoderID = canivore.id(5);

    // Intake
    public static final CANDevice intakeMotorID = rio.id(6);
    public static final CANDevice intakeSensorID = rio.id(6);
    public static final SolenoidPort intakePistonID = pnuematicsHub.solenoidPort(0);
    
    // Puncher
    public static final SolenoidPort puncherPistonID = pnuematicsHub.solenoidPort(1);

    // RIO
    public static final PWMPort ledPort = PWMPort.port(0);
    public static final DIOPort button1 = DIOPort.port(0);
    public static final DIOPort button2 = DIOPort.port(0);
    public static final DIOPort button3 = DIOPort.port(0);
}
