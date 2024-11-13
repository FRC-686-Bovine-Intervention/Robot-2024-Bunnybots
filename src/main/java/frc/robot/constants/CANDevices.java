package frc.robot.constants;

public final class CANDevices {
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

    // Arm
    public static final int armMotorID = 0;
    public static final int armEncoderID = 0;

    // Misc
    public static final int pigeonCanID = 0;

    public static final double minCanUpdateRate = 4.0;
}
