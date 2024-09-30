package frc.robot.subsystems.drive;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.controllers.Joystick;

public class SwerveJoysticks {

    public static Supplier<ProcessedJoysticks> process(Joystick joystick, DoubleSupplier turnSupplier, boolean squareLinearInputs, boolean squareTurnInputs, double slewRatePerSec) {
        return new Supplier<ProcessedJoysticks>() {

            private final SlewRateLimiter slewRateLimiter = new SlewRateLimiter(slewRatePerSec);
            private final Joystick m_joystick = new Joystick(joystick.y(), joystick.x().invert());

            @Override
			public ProcessedJoysticks get() {
                double x = applyDeadband(m_joystick.x().getAsDouble());
                double y = applyDeadband(m_joystick.y().getAsDouble());
                double turn = applyDeadband(turnSupplier.getAsDouble());

                double linearMagnitude = Math.hypot(x, y);
                double linearAngleRadians = Math.atan2(y, x);

                // apply non-lineariy for increased sensitivity for smaller movements
                if (squareLinearInputs) {
                    linearMagnitude = linearMagnitude * linearMagnitude;
                }
                if (squareTurnInputs) {
                    // do not square turn inputs for turns controlled by a PID
                    turn = Math.copySign(turn*turn, turn);
                }

                // limit to unit circle
                linearMagnitude = MathUtil.clamp(linearMagnitude, -1.0, +1.0);
                linearMagnitude = slewRateLimiter.calculate(linearMagnitude);

                return new ProcessedJoysticks(linearMagnitude, linearAngleRadians, turn);
            }
        };
    }

    private static double applyDeadband(double in) {
        double out = 0;
        double deadband = DriveConstants.driveJoystickDeadbandPercent;
        if (Math.abs(in) > deadband) {
            out = Math.copySign((Math.abs(in) - deadband) / (1 - deadband), in);
        }
        return out;
    }


    public static class ProcessedJoysticks {
        double linearMagnitude;
        double linearAngleRadians;
        double turn;

        ProcessedJoysticks(double linearMagnitude, double linearAngleRadians, double turn) {
            this.linearMagnitude = linearMagnitude;
            this.linearAngleRadians = linearAngleRadians;
            this.turn = turn;
        }

        public double getX() {
            return linearMagnitude * Math.cos(linearAngleRadians);
        }
    
        public double getY() {
            return linearMagnitude * Math.sin(linearAngleRadians);
        }
    
        public double getTurn() {
            return turn;
        }
    }        


}