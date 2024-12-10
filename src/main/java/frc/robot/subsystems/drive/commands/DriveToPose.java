package frc.robot.subsystems.drive.commands;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.drive.Drive;
import frc.util.LoggedTunableMeasure;
import frc.util.loggerUtil.tunables.LoggedTunablePID;
import frc.util.wpilib.ProfiledPIDController;

@Deprecated
public class DriveToPose extends Command {
    private final Drive drive;
    private final Supplier<Pose2d> targetSupplier;
    private Pose2d target;
    private Pose2d initialPose;
    private Vector<N2> targetToInitialPoseVector;

    private State currentLinearState;
    private final State goalLinearState = new State(0,0);
    private static final LoggedTunableMeasure<LinearVelocityUnit> linearMaxVelocity = new LoggedTunableMeasure<>("Auto Drive/Linear/Profile/Max Velocity", MetersPerSecond.of(2));
    private static final LoggedTunableMeasure<LinearAccelerationUnit> linearMaxAcceleration = new LoggedTunableMeasure<>("Auto Drive/Linear/Profile/Max Acceleration", MetersPerSecondPerSecond.of(4));
    private final TrapezoidProfile maxLinearProfile = new TrapezoidProfile(
        new Constraints(
            linearMaxVelocity.get(),
            linearMaxAcceleration.get()
        )
    );
    private TrapezoidProfile linearProfile = maxLinearProfile;
    private static final LoggedTunablePID linearPIDConsts = new LoggedTunablePID(
        "Auto Drive/Linear/PID",
        2,
        0,
        0
    );
    private final PIDController linearPID = new PIDController(0, 0, 0);
    
    private static final LoggedTunableMeasure<AngularVelocityUnit> angularMaxVelocity = new LoggedTunableMeasure<>("Auto Drive/Angular/Profile/Max Velocity", DegreesPerSecond.of(180));
    private static final LoggedTunableMeasure<AngularAccelerationUnit> angularMaxAcceleration = new LoggedTunableMeasure<>("Auto Drive/Angular/Profile/Max Acceleration", DegreesPerSecondPerSecond.of(360));
    private static final LoggedTunablePID angularPIDConsts = new LoggedTunablePID(
        "Auto Drive/Angular/PID",
        2,
        0,
        0
    );
    private final ProfiledPIDController angularPID = new ProfiledPIDController(
        0,
        0,
        0,
        new Constraints(
            angularMaxVelocity.get(),
            angularMaxAcceleration.get()
        )
    );

    public DriveToPose(Drive drive, Supplier<Pose2d> targetSupplier) {
        this.drive = drive;
        this.targetSupplier = targetSupplier;
        setName("Drive to " + this.target);
        addRequirements(this.drive.translationSubsystem, this.drive.rotationalSubsystem);
        angularPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        target = targetSupplier.get();
        if (linearPIDConsts.hasChanged(hashCode())) {
            linearPIDConsts.update(linearPID);
        }
        if (angularPIDConsts.hasChanged(hashCode())) {
            angularPIDConsts.update(angularPID);
        }


        initialPose = drive.getPose();
        var startingOmega = drive.getFieldMeasuredSpeeds().omegaRadiansPerSecond;
        var startingAngularState = new State(
            initialPose.getRotation().getRadians(),
            startingOmega
        );
        var goalAngularState = new State(
            target.getRotation().getRadians(),
            0
        );
        angularPID.reset(startingAngularState);
        angularPID.setGoal(goalAngularState);
        angularPID.calculateWithoutProgressing();
        var angularTime = angularPID.getTotalTime();
        
        var velocityVector = VecBuilder.fill(drive.getFieldMeasuredSpeeds().vxMetersPerSecond, drive.getFieldMeasuredSpeeds().vyMetersPerSecond);
        targetToInitialPoseVector = initialPose.getTranslation().minus(target.getTranslation()).toVector().unit();
        var startingVelocity = velocityVector.dot(targetToInitialPoseVector);
        var linearDistance = target.minus(initialPose).getTranslation().getNorm();
        var state = new State(
            linearDistance,
            startingVelocity
        );
        var maxLinearVelocity = linearDistance / angularTime;
        if (maxLinearVelocity < linearMaxVelocity.in(MetersPerSecond)) {
            linearProfile = new TrapezoidProfile(
                new Constraints(
                    maxLinearVelocity,
                    linearMaxAcceleration.in(MetersPerSecondPerSecond)
                )
            );
        } else {
            linearProfile = maxLinearProfile;
        }
        currentLinearState = state;

        Logger.recordOutput("Auto drive/Target", target);
        Logger.recordOutput("Auto drive/Angular/Time", angularTime);
        Logger.recordOutput("Auto drive/Linear/Profile/Max Velocity", maxLinearVelocity);
        Logger.recordOutput("Auto drive/Linear/Profile/Total Distance", linearDistance);
    }

    @Override
    public void execute() {
        var currentPose = drive.getPose();
        var angularPIDOut = angularPID.calculate(currentPose.getRotation().getRadians());

        currentLinearState = linearProfile.calculate(
            RobotConstants.rioUpdatePeriodSecs,
            currentLinearState,
            goalLinearState
        );
        var setpointTrans = new Translation2d(targetToInitialPoseVector.times(currentLinearState.position)).plus(target.getTranslation());
        var linearError = setpointTrans.minus(currentPose.getTranslation()).toVector();
        var linearPIDOut = -linearPID.calculate(linearError.norm(), 0);
        var linearVectorPIDOut = linearError.unit().times(linearPIDOut);
        var linearOut = linearVectorPIDOut.plus(targetToInitialPoseVector.times(currentLinearState.velocity));

        var fieldSpeeds = new ChassisSpeeds(
            linearOut.get(0),
            linearOut.get(1),
            angularPIDOut + angularPID.getSetpoint().velocity
        );

        drive.runFieldSpeeds(fieldSpeeds);
        Logger.recordOutput("Auto drive/Linear/State/Position", currentLinearState.position);
        Logger.recordOutput("Auto drive/Linear/State/Velocity", currentLinearState.velocity);
        Logger.recordOutput("Auto drive/Linear/Error", linearError);
        Logger.recordOutput("Auto drive/Linear/PID Out", linearPIDOut);
        Logger.recordOutput("Auto drive/Setpoint", new Pose2d(setpointTrans, new Rotation2d(angularPID.getSetpoint().position)));
    }

    @Override
    public void end(boolean interrupted) {
        drive.runRobotSpeeds(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
