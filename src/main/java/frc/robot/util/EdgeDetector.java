package frc.robot.util;

import java.util.function.BooleanSupplier;

public class EdgeDetector {
    private final BooleanSupplier source;

    public EdgeDetector(BooleanSupplier source) {
        this.source = source;
    }

    private boolean val;
    private boolean prevVal;
    private boolean risingEdge;
    private boolean fallingEdge;

    public void update() {
        val = source.getAsBoolean();
        risingEdge = val && !prevVal;
        fallingEdge = !val && prevVal;
        prevVal = val;
    }

    public boolean getValue() {
        return val;
    }
    public boolean risingEdge() {
        return risingEdge;
    }
    public boolean fallingEdge() {
        return fallingEdge;
    }
    public boolean changed() {
        return risingEdge || fallingEdge;
    }
}
