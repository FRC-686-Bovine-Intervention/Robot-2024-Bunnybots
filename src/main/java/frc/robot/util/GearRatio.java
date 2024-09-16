package frc.robot.util;

import edu.wpi.first.math.util.Units;

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

    public Wheel wheelRadius(double radius) {
        return new Wheel(radius, this);
    }
    public Wheel wheelDiameter(double diameter) {
        return wheelRadius(diameter / 2);
    }
    public Wheel wheelCircumference(double circumference) {
        return wheelDiameter(circumference / Math.PI);
    }

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
        private final double radius;
        private final GearRatio ratio;
        private final GearRatio inverseRatio;

        private Wheel(double radius, GearRatio ratio) {
            this.radius = radius;
            this.ratio = ratio;
            this.inverseRatio = this.ratio.inverse();
        }

        public double surfacePerRad() {
            return ratio.ratio() * radius;
        }
        public double surfacePerRot() {
            return Units.rotationsToRadians(surfacePerRad());
        }

        public double radPerSurface() {
            return inverseRatio.ratio() / radius;
        }
        public double rotPerSurface() {
            return Units.radiansToRotations(radPerSurface());
        }

        public double radsToSurface(double rads) {
            return surfacePerRad() * rads;
        }
        public double rotsToSurface(double rots) {
            return surfacePerRot() * rots;
        }

        public double surfaceToRads(double surface) {
            return radPerSurface() * surface;
        }
        public double surfaceToRots(double surface) {
            return rotPerSurface() * surface;
        }
    }
}
