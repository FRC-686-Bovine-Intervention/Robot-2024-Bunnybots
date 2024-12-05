package frc.util;

import static edu.wpi.first.units.Units.Meters;

import java.util.Arrays;
import java.util.stream.IntStream;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

public class PoseBoundingBoxUtil {
    public static interface BoundingBox {
        public boolean withinBounds(Translation2d trans);
        public default boolean withinBounds(Pose2d pose) {
            return withinBounds(pose.getTranslation());
        }

        public static RectangularBoundingBox rectangle(Translation2d corner1, Translation2d corner2) {
            return new RectangularBoundingBox(corner1, corner2);
        }
        public static CircularBoundingBox circle(Translation2d center, Distance radius) {
            return new CircularBoundingBox(center, radius);
        }

        public default BoundingBox or(BoundingBox... boxes) {
            return new OrBox(this).or(boxes);
        }
        public default BoundingBox and(BoundingBox... boxes) {
            return new AndBox(this).and(boxes);
        }
        public default BoundingBox not() {
            return new NotBox(this);
        }
    }
    // Primitives
    public static class RectangularBoundingBox implements BoundingBox {
        private final Translation2d bottomLeftCorner;
        private final Translation2d topRightCorner;

        public RectangularBoundingBox(Translation2d corner1, Translation2d corner2) {
            this.bottomLeftCorner = new Translation2d(Math.min(corner1.getX(), corner2.getX()), Math.min(corner1.getY(), corner2.getY()));
            this.topRightCorner = new Translation2d(Math.max(corner1.getX(), corner2.getX()), Math.max(corner1.getY(), corner2.getY()));
        }

        @Override
        public boolean withinBounds(Translation2d trans) {
            return 
                trans.getX() >= bottomLeftCorner.getX()
                && trans.getX() <= topRightCorner.getX()
                && trans.getY() >= bottomLeftCorner.getY()
                && trans.getY() <= topRightCorner.getY()
            ;
        }
    }
    public static class CircularBoundingBox implements BoundingBox {
        private final Translation2d center;
        private final Distance radius;

        public CircularBoundingBox(Translation2d center, Distance radius) {
            this.center = center;
            this.radius = radius;
        }

        @Override
        public boolean withinBounds(Translation2d trans) {
            return center.getDistance(trans) <= radius.in(Meters);
        }
    }

    // Compound
    public static class OrBox implements BoundingBox {
        private final BoundingBox[] boxes;

        public OrBox(BoundingBox... boxes) {
            this.boxes = boxes;
        }

        @Override
        public boolean withinBounds(Translation2d trans) {
            return Arrays.stream(boxes).anyMatch((box) -> box.withinBounds(trans));
        }

        @Override
        public OrBox or(BoundingBox... boxes) {
            BoundingBox[] newBoxes = new BoundingBox[boxes.length + this.boxes.length];
            IntStream.range(0, this.boxes.length).forEach((i) -> newBoxes[i] = this.boxes[i]);
            IntStream.range(0, boxes.length).forEach((i) -> newBoxes[i + this.boxes.length] = boxes[i]);
            return new OrBox(newBoxes);
        }
    }
    public static class AndBox implements BoundingBox {
        private final BoundingBox[] boxes;

        public AndBox(BoundingBox... boxes) {
            this.boxes = boxes;
        }

        @Override
        public boolean withinBounds(Translation2d trans) {
            return Arrays.stream(boxes).allMatch((box) -> box.withinBounds(trans));
        }

        @Override
        public AndBox and(BoundingBox... boxes) {
            BoundingBox[] newBoxes = new BoundingBox[boxes.length + this.boxes.length];
            IntStream.range(0, this.boxes.length).forEach((i) -> newBoxes[i] = this.boxes[i]);
            IntStream.range(0, boxes.length).forEach((i) -> newBoxes[i + this.boxes.length] = boxes[i]);
            return new AndBox(newBoxes);
        }
    }
    public static class NotBox implements BoundingBox {
        private final BoundingBox box;

        public NotBox(BoundingBox box) {
            this.box = box;
        }

        @Override
        public boolean withinBounds(Translation2d trans) {
            return !box.withinBounds(trans);
        }

        @Override
        public BoundingBox not() {
            return box;
        }
    }
}
