package frc.robot.util.led.functions;

import java.util.function.DoubleUnaryOperator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.InterpolationFunction;

@FunctionalInterface
public interface Gradient {
    public Color getColor(double x);

    public static class BasicGradient implements Gradient {
        private final Color[] colors;
        private final InterpolationFunction<Color> interpolationFunction;
        public static enum InterpolationStyle implements InterpolationFunction<Color> {
            Step((double t, Double[] data) -> data[(int)MathUtil.clamp(Math.floor(t*data.length), 0, data.length-1)]),
            Linear((double t, Double[] data) -> {
                if(data.length <= 0) return 0.0;
                t *= (data.length - 1);
                int lower = (int)Math.floor(t);
                int upper = (int)Math.ceil(t);
                t = (t % 1 + 1) % 1;
                return (data[upper] - data[lower]) * t + data[lower];
            }),
            ;
            private final InterpolationFunction<Color> function;
            InterpolationStyle(InterpolationFunction<Double> function) {
                this.function = (double t, Color[] data) -> {
                    Double[] r = new Double[data.length];
                    Double[] g = new Double[data.length];
                    Double[] b = new Double[data.length];
                    for(int i = 0; i < data.length; i++) {
                        var color = data[i];
                        r[i] = color.red;
                        g[i] = color.green;
                        b[i] = color.blue;
                    }
                    return new Color(
                        function.interpolate(t, r), 
                        function.interpolate(t, g), 
                        function.interpolate(t, b)
                    );
                };
            }
            @Override
            public Color interpolate(double t, Color[] data) {return function.interpolate(t, data);}
        }

        public BasicGradient(InterpolationFunction<Color> interpolationFunction, Color... colors) {
            this.colors = colors;
            this.interpolationFunction = interpolationFunction;
        }

        @Override
        public Color getColor(double x) {
            return interpolationFunction.interpolate(x, colors);
        }
    }

    public static final Gradient blackToWhite = (double x) -> new Color(x,x,x);
    public static final Gradient rainbow = (double x) -> {
        final DoubleUnaryOperator sinusoid = (double X) -> Math.max(Math.cos(2 * Math.PI * X) * (2.0 / 3) + (1.0 / 3), 0);
        return new Color(
            sinusoid.applyAsDouble(x - 0.0/3), 
            sinusoid.applyAsDouble(x - 1.0/3), 
            sinusoid.applyAsDouble(x - 2.0/3)
        );
    };
    public static final Gradient hueShift = (double x) -> Color.fromHSV((int)(x * 180),255,255);
}
