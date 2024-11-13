package frc.robot.constants;

public final class CANDevices {
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
    // Drive
    public static final String driveCanBusName = "rio";
    // | Front Left
    public static final int frontLeftDriveMotorID  = 1;
    public static final int frontLeftTurnMotorID   = 1;
    // | Front Right
    public static final int frontRightDriveMotorID  = 2;
    public static final int frontRightTurnMotorID   = 2;
    // | Back Left
    public static final int backLeftDriveMotorID  = 3;
    public static final int backLeftTurnMotorID   = 3;
    // | Back Right
    public static final int backRightDriveMotorID  = 4;
    public static final int backRightTurnMotorID   = 4;

    // Misc
    public static final int pigeonCanID = 0;

    public static final double minCanUpdateRate = 4.0;
}
