package frc.util.led.animation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.GameState;
import frc.util.led.functions.Gradient;
import frc.util.led.functions.InterpolationFunction;
import frc.util.led.functions.WaveFunction;
import frc.util.led.strips.LEDStrip;

public class AutonomousFinishedAnimation extends LEDAnimation {
    private final FlashingAnimation overrunAnimation;
    private final BarAnimation finishedAnimation;
    private final LEDStrip finishedStrip;
    private boolean overrun;

    public AutonomousFinishedAnimation(LEDStrip finishedStrip, LEDStrip overrunStrip) {
        this.finishedStrip = finishedStrip;
        this.finishedAnimation = new BarAnimation(finishedStrip, new Gradient(Color.kGreen));
        this.overrunAnimation = new FlashingAnimation(overrunStrip, WaveFunction.Modulo.frequency(2), InterpolationFunction.step.gradient(Color.kBlack, Color.kRed));
    }

    @Override
    public void apply() {
        if(overrun) {
            if(!DriverStation.isFMSAttached()) {
                overrunAnimation.apply();
            }
        } else if(
            GameState.getInstance().lastEnabledMode.isAutonomous() && 
            GameState.getInstance().AUTONOMOUS_COMMAND_FINISH.isSet() && 
            !GameState.getInstance().AUTONOMOUS_ALLOTTED_TIMESTAMP.hasBeenSince(0)
        ) {
            var until = MathUtil.inverseInterpolate(
                GameState.getInstance().AUTONOMOUS_COMMAND_FINISH.timestamp.getAsDouble(),
                GameState.getInstance().AUTONOMOUS_ALLOTTED_TIMESTAMP.timestamp.getAsDouble(),
                Timer.getFPGATimestamp()
            );
            finishedStrip.apply(Color.kBlack);
            finishedAnimation.setPos(1 - until);
            finishedAnimation.apply();
        }
    }

    public void setOverrun(boolean overrun) {
        this.overrun = overrun;
    }
}
