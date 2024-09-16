package frc.robot.util.led.functions;

@FunctionalInterface
public interface TilingFunction {
    public double tile(double x);

    public static final TilingFunction Modulo = (double x) -> (x % 1 + 1) % 1;
    public static final TilingFunction Sawtooth = (double x) -> 1 - Math.abs(((x % 2 + 2) % 2) - 1);
    public static final TilingFunction Sinusoidal = (double x) -> 0.5 * (Math.sin(Math.PI * (x - 0.5)) + 1);
}
