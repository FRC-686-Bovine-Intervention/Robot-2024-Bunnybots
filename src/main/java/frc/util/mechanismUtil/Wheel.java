package frc.util.mechanismUtil;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class Wheel {
    private final Distance radius;

    private Wheel(Distance radius) {
        this.radius = radius;
    }

    public static Wheel radius(Distance radius) {
        return new Wheel(radius);
    }
    public static Wheel diameter(Distance diameter) {
        return radius(diameter.divide(2));
    }
    public static Wheel circumference(Distance circumference) {
        return diameter(circumference.divide(Math.PI));
    }

    public Distance radius() {
        return radius;
    }

    public Distance surfacePer(AngleUnit angleUnits) {
        return angleToSurface(angleUnits.one());
    }
    public Angle anglePer(DistanceUnit distUnits) {
        return surfaceToAngle(distUnits.one());
    }

    public Distance angleToSurface(Angle angle) {
        return radius.times(angle.in(Radians));
    }
    public LinearVelocity angularVelocityToSurfaceVelocity(AngularVelocity angularVelocity) {
        return radius.per(Second).times(angularVelocity.in(RadiansPerSecond));
    }
    public Angle surfaceToAngle(Distance surface) {
        return Radians.of(surface.baseUnitMagnitude() / radius.baseUnitMagnitude());
    }
    public AngularVelocity surfaceVelocityToAngularVelocity(LinearVelocity surfaceVelocity) {
        return RadiansPerSecond.of(surfaceVelocity.baseUnitMagnitude() / radius.per(Second).baseUnitMagnitude());
    }
}
