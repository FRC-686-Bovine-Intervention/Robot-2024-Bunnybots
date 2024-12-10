package frc.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import java.util.Arrays;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

public class MathExtraUtil {
    public static double average(double... a) {
        return Arrays.stream(a).average().orElse(0);
    }
    @SafeVarargs
    @SuppressWarnings("unchecked")
    public static <U extends Unit> Measure<U> average(Measure<U>... a) {
        return (Measure<U>)a[0].unit().ofBaseUnits(Arrays.stream(a).mapToDouble((measure) -> measure.baseUnitMagnitude()).average().orElse(0));
    }

    public static Rotation2d backwards(Rotation2d rotation) {
        return new Rotation2d(-rotation.getCos(), -rotation.getSin());
    }

    public static Rotation2d rotationFromVector(Vector<N2> vec) {
        return new Rotation2d(vec.get(0), vec.get(1));
    }
    public static Vector<N2> vectorFromRotation(Rotation2d rot) {
        return VecBuilder.fill(rot.getCos(), rot.getSin());
    }

    public static Vector<N2> vectorFromSpeeds(ChassisSpeeds speeds) {
        return VecBuilder.fill(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    }
    public static Translation2d translationFromSpeeds(ChassisSpeeds speeds) {
        return new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    }
    public static ChassisSpeeds speedsFromVector2D(Vector<N2> vector) {
        return new ChassisSpeeds(vector.get(0), vector.get(1), 0);
    }
    public static ChassisSpeeds speedsFromVector3D(Vector<N3> vector) {
        return new ChassisSpeeds(vector.get(0), vector.get(1), vector.get(2));
    }
    public static ChassisSpeeds speedsFromTranslation(Translation2d translation) {
        return new ChassisSpeeds(translation.getX(), translation.getY(), 0);
    }

    public static boolean isNear(Pose2d expected, Pose2d actual, double linearTolerance, double angularTolerance) {
        return isNear(expected.getTranslation(), actual.getTranslation(), linearTolerance) && isNear(expected.getRotation(), actual.getRotation(), angularTolerance);
    }
    public static boolean isNear(Pose2d expected, Pose2d actual, Measure<DistanceUnit> linearTolerance, Measure<AngleUnit> angularTolerance) {
        return isNear(expected.getTranslation(), actual.getTranslation(), linearTolerance) && isNear(expected.getRotation(), actual.getRotation(), angularTolerance);
    }
    public static boolean isNear(Translation2d expected, Translation2d actual, double tolerance) {
        return actual.getDistance(expected) <= tolerance;
    }
    public static boolean isNear(Translation2d expected, Translation2d actual, Measure<DistanceUnit> tolerance) {
        return isNear(expected, actual, tolerance.in(Meters));
    }
    public static boolean isNear(Rotation2d expected, Rotation2d actual, double tolerance) {
        return Math.abs(actual.minus(expected).getRadians()) <= tolerance;
    }
    public static boolean isNear(Rotation2d expected, Rotation2d actual, Measure<AngleUnit> tolerance) {
        return isNear(expected, actual, tolerance.in(Radians));
    }
    public static boolean isNear(ChassisSpeeds expected, ChassisSpeeds actual, double linearTolerance, double angularTolerance) {
        var bol = isNear(new Translation2d(expected.vxMetersPerSecond, expected.vyMetersPerSecond), new Translation2d(actual.vxMetersPerSecond, actual.vyMetersPerSecond), linearTolerance) && MathUtil.isNear(expected.omegaRadiansPerSecond, actual.omegaRadiansPerSecond, angularTolerance);
        return bol;
    }
    public static <U extends Unit> boolean isNear(Measure<U> expected, Measure<U> actual, Measure<U> tolerance) {
        return MathUtil.isNear(expected.baseUnitMagnitude(), actual.baseUnitMagnitude(), tolerance.baseUnitMagnitude());
    }

    public static boolean isWithin(double value, double min, double max) {
        return value >= min && value <= max;
    }
    public static <U extends Unit> boolean isWithin(Measure<U> value, Measure<U> min, Measure<U> max) {
        return isWithin(value.baseUnitMagnitude(), min.baseUnitMagnitude(), max.baseUnitMagnitude());
    }

    @SuppressWarnings("unchecked")
    public static <U extends Unit> Measure<U> interpolate(Measure<U> start, Measure<U> end, double t) {
        return (Measure<U>)start.unit().ofBaseUnits(MathUtil.interpolate(start.baseUnitMagnitude(), end.baseUnitMagnitude(), t));
    }
    public static <U extends Unit> double inverseInterpolate(Measure<U> start, Measure<U> end, Measure<U> t) {
        return MathUtil.inverseInterpolate(start.baseUnitMagnitude(), end.baseUnitMagnitude(), t.baseUnitMagnitude());
    }

    public static Matrix<N2, N2> rotationMatrix(Rotation2d rot) {
        return MatBuilder.fill(Nat.N2(), Nat.N2(),
			+rot.getCos(), -rot.getSin(),
            +rot.getSin(), +rot.getCos()
		);
    }
}
