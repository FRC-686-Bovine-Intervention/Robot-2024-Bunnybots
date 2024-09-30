package frc.robot.util;

@FunctionalInterface
public interface InterpolationFunction<T> {
    T interpolate(double t, T[] data);
}
