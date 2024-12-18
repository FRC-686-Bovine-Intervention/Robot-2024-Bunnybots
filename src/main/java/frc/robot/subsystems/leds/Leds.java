package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Robot;
import frc.robot.constants.HardwareDevices;
import frc.util.VirtualSubsystem;
import frc.util.led.animation.AllianceColorAnimation;
import frc.util.led.animation.AutonomousFinishedAnimation;
import frc.util.led.animation.FillAnimation;
import frc.util.led.animation.FlashingAnimation;
import frc.util.led.animation.StatusLightAnimation;
import frc.util.led.functions.InterpolationFunction;
import frc.util.led.functions.WaveFunction;
import frc.util.led.strips.hardware.AddressableStrip;
import frc.util.led.strips.hardware.HardwareStrip;

public class Leds extends VirtualSubsystem {
    private static Leds instance;
    public static Leds getInstance() {if(instance == null) {instance = new Leds();} return instance;}
    
    private final HardwareStrip hardwareStrip;

    private final Notifier loadingNotifier;

    public Leds() {
        System.out.println("[Init Leds] Instantiating Leds");
        hardwareStrip = new AddressableStrip(HardwareDevices.ledPort, 57);

        var rightStrip = hardwareStrip.substrip(0, 19);
        var backStrip = hardwareStrip.substrip(19, 38);
        var leftStrip = hardwareStrip.substrip(38, 57).reverse();

        var sideStrips = leftStrip.parallel(rightStrip);
        var sideStripTips = sideStrips.substrip(15).concat(backStrip.substrip(5, 13));
        
        var backRightStrip = backStrip.substrip(0, 10);
        var backLeftStrip = backStrip.substrip(9).reverse();
        
        var fullLeftStrip = leftStrip.concat(backLeftStrip);
        var fullRightStrip = rightStrip.concat(backRightStrip);
        
        var fullSideStrips = fullLeftStrip.parallel(fullRightStrip);

        estopped = new FillAnimation(hardwareStrip, Color.kRed);
        allianceColorAnimation = new AllianceColorAnimation(fullSideStrips);
        driverStationConnection = new StatusLightAnimation(sideStrips.substrip(0, 2), Color.kOrange, Color.kGreen);
        lAprilConnection = new StatusLightAnimation(sideStrips.substrip(2, 3), Color.kOrange, Color.kGreen);
        rAprilConnection = new StatusLightAnimation(sideStrips.substrip(3, 4), Color.kOrange, Color.kGreen);
        nVisionConnection = new StatusLightAnimation(sideStrips.substrip(4, 5), Color.kOrange, Color.kGreen);
        noteSecured = new FillAnimation(sideStripTips, Color.kGreen);
        visionAcquired = new FillAnimation(sideStripTips, Color.kOrange);
        visionLocked = new FillAnimation(sideStripTips, Color.kPurple);
        defenseSpin = new FlashingAnimation(hardwareStrip, WaveFunction.Sinusoidal, InterpolationFunction.linear.gradient(Color.kBlack, Color.kYellow));
        humanPlayerFlash = new FlashingAnimation(hardwareStrip, WaveFunction.Sawtooth, InterpolationFunction.linear.gradient(Color.kBlack, Color.kWhite));
        noteAcquired = new FlashingAnimation(hardwareStrip, WaveFunction.Sawtooth, InterpolationFunction.linear.gradient(Color.kBlack, Color.kGreen));
        autonomousFinishedAnimation = new AutonomousFinishedAnimation(sideStrips, hardwareStrip);

        loadingNotifier = new Notifier(() -> {
            synchronized(this) {
                hardwareStrip.apply(
                    InterpolationFunction.linear.gradient(
                        Color.kBlack,
                        Color.kDimGray
                    )
                    .apply(
                        WaveFunction.Sinusoidal.applyAsDouble(
                            System.currentTimeMillis() / 1000.0
                        )
                    )
                );
                hardwareStrip.refresh();
            }
        });
        System.out.println("[Init Leds] Starting Loading Notifier");
        loadingNotifier.startPeriodic(Robot.defaultPeriodSecs);
    }

    public final FillAnimation estopped;
    public final AllianceColorAnimation allianceColorAnimation;
    public final StatusLightAnimation driverStationConnection;
    public final StatusLightAnimation lAprilConnection;
    public final StatusLightAnimation rAprilConnection;
    public final StatusLightAnimation nVisionConnection;
    public final FlashingAnimation noteAcquired;
    public final FillAnimation noteSecured;
    public final FillAnimation visionAcquired;
    public final FillAnimation visionLocked;
    public final FlashingAnimation defenseSpin;
    public final FlashingAnimation humanPlayerFlash;
    public final AutonomousFinishedAnimation autonomousFinishedAnimation;

    private int skippedFrames = 0;
    private static final int frameSkipAmount = 15;

    @Override
    public void periodic() {
        driverStationConnection.setStatus(DriverStation.isDSAttached());
    }

    @Override
    public synchronized void postCommandPeriodic() {
        if(skippedFrames < frameSkipAmount) {
            skippedFrames++;
            return;
        }
        loadingNotifier.stop();

        // Default alliance color scrolling
        allianceColorAnimation.apply();

        if(DriverStation.isDisabled()) {
            driverStationConnection.apply();
            lAprilConnection.apply();
            rAprilConnection.apply();
            nVisionConnection.apply();
        }

        visionAcquired.applyIfFlagged();
        visionLocked.applyIfFlagged();
        noteSecured.applyIfFlagged();

        defenseSpin.applyIfFlagged();

        humanPlayerFlash.applyIfFlagged();

        noteAcquired.applyIfFlagged();

        autonomousFinishedAnimation.applyIfFlagged();

        estopped.applyIfFlagged();

        //TODO: End game notification
        hardwareStrip.refresh();
    }
}
