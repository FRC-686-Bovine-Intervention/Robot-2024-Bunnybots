package frc.robot.subsystems.drive.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.AllianceFlipUtil.FieldFlipType;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.controllers.Joystick;

public class FieldOrientedDrive extends Command {

	// private final Drive drive;
    // private final Supplier<ChassisSpeeds> fieldRelativeSupplier;
	// private final ToDoubleBiFunction<Rotation2d, Optional<Rotation2d>> headingToTurnRate;
	// private Optional<Rotation2d> override;

	// public static LazyOptional<Rotation2d> headingFromJoystick(Joystick joystick, Supplier<Rotation2d[]> snapPointsSupplier, Supplier<Rotation2d> forwardDirectionSupplier) {
	// 	return new LazyOptional<Rotation2d>() {
	// 		private final Timer preciseTurnTimer = new Timer();
	// 		private final double preciseTurnTimeThreshold = 0.5;
	// 		private Optional<Rotation2d> outputFilter(Rotation2d i) {
	// 			return Optional.of(i.minus(forwardDirectionSupplier.get()));
	// 		}
	// 		@Override
	// 		public Optional<Rotation2d> get() {
	// 			if(joystick.magnitude() == 0) {
	// 				preciseTurnTimer.restart();
	// 				return Optional.empty();
	// 			}
	// 			var joyVec = new Translation2d(joystick.x().getAsDouble(), joystick.y().getAsDouble());
	// 			joyVec = SpectatorType.getCurrentType().toField(joyVec);
	// 			Rotation2d joyHeading = AllianceFlipUtil.apply(Rotation2d.fromRadians(Math.atan2(joyVec.getY(), joyVec.getX())), FieldFlipType.CenterPointFlip);
	// 			if(preciseTurnTimer.hasElapsed(preciseTurnTimeThreshold)) {
	// 				return outputFilter(joyHeading);
	// 			}
	// 			var snapPoints = snapPointsSupplier.get();
	// 			int smallestDistanceIndex = 0;
	// 			double smallestDistance = Double.MAX_VALUE;
	// 			for(int i = 0; i < snapPoints.length; i++) {
	// 				var dist = Math.abs(joyHeading.minus(AllianceFlipUtil.apply(snapPoints[i])).getRadians());
	// 				if(dist < smallestDistance) {
	// 					smallestDistance = dist;
	// 					smallestDistanceIndex = i;
	// 				}
	// 			}
	// 			return outputFilter(AllianceFlipUtil.apply(snapPoints[smallestDistanceIndex]));
	// 		}
	// 	};
	// }

	// public static LazyOptional<Rotation2d> pointTo(Supplier<Optional<Translation2d>> posToPointTo, Supplier<Rotation2d> forward) {
    //     return () -> posToPointTo.get().map((pointTo) -> Rotation2d.fromRadians(Math.atan2(
	// 		pointTo.getY() - RobotState.getInstance().getPose().getY(),
	// 		pointTo.getX() - RobotState.getInstance().getPose().getX()
	// 	)).minus(forward.get()));
    // }

	// public static ToDoubleBiFunction<Rotation2d, Optional<Rotation2d>> pidControlledHeading(Supplier<Optional<Rotation2d>> headingSupplier) {
	// 	return new ToDoubleBiFunction<Rotation2d, Optional<Rotation2d>>() {
	// 		private final PIDController headingPID = new PIDController(DriveConstants.headingKp, DriveConstants.headingKi, DriveConstants.headingKd);
	// 		{
	// 			headingPID.enableContinuousInput(-Math.PI, Math.PI);  // since gyro angle is not limited to [-pi, pi]
	// 			headingPID.setTolerance(DriveConstants.headingTolerance);
	// 		}
	// 		private Rotation2d desiredHeading;
	// 		@Override
	// 		public double applyAsDouble(Rotation2d robotHeading, Optional<Rotation2d> override) {
	// 			override.ifPresent((r) -> desiredHeading = r);
	// 			headingSupplier.get().ifPresent((r) -> desiredHeading = r);
	// 			double turnInput = headingPID.calculate(robotHeading.getRadians(), desiredHeading.getRadians());
	// 			turnInput = headingPID.atSetpoint() ? 0 : turnInput;
	// 			turnInput = MathUtil.clamp(turnInput, -0.5, +0.5);
	// 			return turnInput * DriveConstants.maxTurnRateRadiansPerSec;
	// 		}
	// 	};
	// }

	// public static Command epidControlledHeading(Supplier<Optional<Rotation2d>> headingSupplier, Drive.Rotational rotation) {
	// 	return new Command() {
	// 		private final PIDController headingPID = new PIDController(DriveConstants.headingKp, DriveConstants.headingKi, DriveConstants.headingKd);
	// 		{
	// 			headingPID.enableContinuousInput(-Math.PI, Math.PI);  // since gyro angle is not limited to [-pi, pi]
	// 			headingPID.setTolerance(DriveConstants.headingTolerance);
	// 		}
	// 		private Rotation2d desiredHeading;
	// 		@Override
	// 		public void initialize() {
	// 			desiredHeading = rotation.drive.getPose().getRotation();
	// 		}
	// 		@Override
	// 		public void execute() {
	// 			headingSupplier.get().ifPresent((r) -> desiredHeading = r);
	// 			double turnInput = headingPID.calculate(rotation.drive.getPose().getRotation().getRadians(), desiredHeading.getRadians());
	// 			turnInput = headingPID.atSetpoint() ? 0 : turnInput;
	// 			turnInput = MathUtil.clamp(turnInput, -0.5, +0.5);
	// 			rotation.driveVelocity(turnInput);
	// 		}
	// 		@Override
	// 		public void end(boolean interrupted) {
	// 			rotation.driveVelocity(0);
	// 		}
	// 	};
	// }

	private static final LoggedTunableNumber spectatorType = new LoggedTunableNumber("Spectator Type", 1);
	public static enum SpectatorType {
		Comp(
			MatBuilder.fill(Nat.N2(), Nat.N2(),
				+0,+1,
				-1,+0
			)
		),
		Spectator(
			MatBuilder.fill(Nat.N2(), Nat.N2(),
				+1,+0,
				+0,+1
			)
		),
		ISpectator(
			MatBuilder.fill(Nat.N2(), Nat.N2(),
				-1,+0,
				+0,-1
			)
		),
		;
		private final Matrix<N2, N2> spectatorToField;
		private final Matrix<N2, N2> fieldToSpectator;
		SpectatorType(Matrix<N2, N2> spectatorToField) {
			this.spectatorToField = spectatorToField;
			this.fieldToSpectator = this.spectatorToField.inv();
		}
		public Vector<N2> toField(Vector<N2> vec) {
			return new Vector<N2>(spectatorToField.times(vec));
		}
		public Vector<N2> toSpectator(Vector<N2> vec) {
			return new Vector<N2>(fieldToSpectator.times(vec));
		}
		
		public Vector<N2> getForwardFieldRel() {
			return toField(VecBuilder.fill(0, 1));
		}

		public static SpectatorType getCurrentType() {
			if(DriverStation.getMatchType() != MatchType.None) return Comp;
			return SpectatorType.values()[MathUtil.clamp((int) spectatorType.get(), 0, values().length - 1)];
		}
	}
	public static Supplier<ChassisSpeeds> joystickSpectatorToFieldRelative(Joystick translationalJoystick, BooleanSupplier precisionSupplier) {
		return () -> {
			var fieldVec = SpectatorType.getCurrentType().toField(
				translationalJoystick.toVector()
				.times(
					DriveConstants.maxDriveSpeedMetersPerSec * 
					(precisionSupplier.getAsBoolean() ? DriveConstants.precisionLinearMultiplier : 1)
				)
			);
			return AllianceFlipUtil.applyFieldRelative(
				new ChassisSpeeds(
					fieldVec.get(0),
					fieldVec.get(1),
					0
				),
				FieldFlipType.CenterPointFlip
			);
		};
	}

	// public FieldOrientedDrive(Drive drive, Supplier<ChassisSpeeds> fieldRelativeSupplier, ToDoubleBiFunction<Rotation2d, Optional<Rotation2d>> headingToTurnRate) {
	// 	addRequirements(drive);
	// 	setName("DriveWithCustomFlick");
	// 	this.drive = drive;
    //     this.fieldRelativeSupplier = fieldRelativeSupplier;
    //     this.headingToTurnRate = headingToTurnRate;
	// }

	// @Override
	// public void initialize() {
	// 	override = Optional.of(drive.getPose().getRotation());
	// }

	// @Override
	// public void execute() {
	// 	var speeds = fieldRelativeSupplier.get();
	// 	var heading = drive.getPose().getRotation();
	// 	speeds.omegaRadiansPerSecond = headingToTurnRate.applyAsDouble(heading, override);
	// 	override = Optional.empty();

	// 	speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, heading);

	// 	drive.driveVelocity(speeds);
	// }

	// @Override
	// public void end(boolean interrupted) {
	// 	drive.stop();
	// }
}
