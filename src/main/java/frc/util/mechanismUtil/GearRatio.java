package frc.util.mechanismUtil;

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
}
