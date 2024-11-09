package frc.util;

import java.util.function.BooleanSupplier;

public class SuppliedEdgeDetector {
    private final BooleanSupplier source;
    private final EdgeDetector edgeDetector = new EdgeDetector();

    public SuppliedEdgeDetector(BooleanSupplier source) {
        this.source = source;
    }

    public void update() {
        edgeDetector.update(source.getAsBoolean());
    }

    public boolean getValue() {
        return edgeDetector.getValue();
    }
    public boolean risingEdge() {
        return edgeDetector.risingEdge();
    }
    public boolean fallingEdge() {
        return edgeDetector.fallingEdge();
    }
    public boolean changed() {
        return edgeDetector.changed();
    }
}
