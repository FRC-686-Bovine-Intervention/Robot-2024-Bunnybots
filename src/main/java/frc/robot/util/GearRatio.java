package frc.robot.util;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class GearRatio {
    private final double ratio;

    public GearRatio() {
        this(1);
    }

    private GearRatio(double ratio) {
        this.ratio = ratio;
    }

    public double ratio() {
        return ratio;
    }
    
    public double apply(double source) {
        return source * ratio;
    }
    
    public GearRatio inverse() {
        return new GearRatio(1/ratio);
    }

    public GearRatio concat(GearRatio other) {
        return new GearRatio(ratio * other.ratio);
    }

    public Gear gear(double toothCount) {
        return new Gear(toothCount, this);
    }

    public GearRatio planetary(double ratio) {
        return new GearRatio(this.ratio * ratio);
    }

    public Chain sprocket(double toothCount) {
        return new Chain(toothCount, this);
    }

    // public Wheel wheelRadius(Measure<Distance> radius) {
    //     return new Wheel(radius);
    // }
    // public Wheel wheelDiameter(Measure<Distance> diameter) {
    //     return wheelRadius(diameter.divide(2));
    // }
    // public Wheel wheelCircumference(Measure<Distance> circumference) {
    //     return wheelDiameter(circumference.divide(Math.PI));
    // }

    public static class Gear {
        private final double toothCount;
        private final GearRatio ratio;

        private Gear(double toothCount, GearRatio ratio) {
            this.toothCount = toothCount;
            this.ratio = ratio;
        }

        public Gear gear(double toothCount) {
            return new Gear(toothCount, new GearRatio(-ratio.ratio * this.toothCount / toothCount));
        }

        public GearRatio axle() {
            return ratio;
        }
    }

    public static class Chain {
        private final double toothCount;
        private final GearRatio ratio;

        private Chain(double toothCount, GearRatio ratio) {
            this.toothCount = toothCount;
            this.ratio = ratio;
        }

        public GearRatio sprocket(double toothCount) {
            return new GearRatio(ratio.ratio * this.toothCount / toothCount);
        }

        public GearRatio inverseSprocket(double toothCount) {
            return sprocket(-toothCount);
        }
    }

    public static class Wheel {
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

        // public double surfacePerRad() {
        //     return ratio.ratio() * radius;
        // }
        // public double surfacePerRot() {
        //     return Units.rotationsToRadians(surfacePerRad());
        // }

        // public double radPerSurface() {
        //     return inverseRatio.ratio() / radius;
        // }
        // public double rotPerSurface() {
        //     return Units.radiansToRotations(radPerSurface());
        // }

        // public double radsToSurface(double rads) {
        //     return surfacePerRad() * rads;
        // }
        // public double rotsToSurface(double rots) {
        //     return surfacePerRot() * rots;
        // }

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
        // public double surfaceToRads(double surface) {
        //     return radPerSurface() * surface;
        // }
        // public double surfaceToRots(double surface) {
        //     return rotPerSurface() * surface;
        // }
    }
}
