package frc.robot.subsystems.leds;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.GameState;
import frc.robot.Robot;
import frc.robot.auto.AutoConstants;
import frc.robot.util.VirtualSubsystem;
import frc.robot.util.led.functions.Gradient.InterpolationStyle;
import frc.robot.util.led.functions.TilingFunction;
import frc.robot.util.led.strips.LEDStrip;
import frc.robot.util.led.strips.hardware.AddressableStrip;
import frc.robot.util.led.strips.hardware.HardwareStrip;

public class Leds extends VirtualSubsystem {
    private static Leds instance;
    public static Leds getInstance() {if(instance == null) {instance = new Leds();} return instance;}
    
    private final HardwareStrip hardwareStrip;
    private final LEDStrip rightStrip;
    private final LEDStrip backStrip;
    private final LEDStrip leftStrip;

    private final LEDStrip sideStrips;
    private final LEDStrip sideStripTips;
    private final LEDStrip dsConnectedStrip;
    private final LEDStrip lAprilConnectedStrip;
    private final LEDStrip rAprilConnectedStrip;
    private final LEDStrip nVisionConnectedStrip;

    private final LEDStrip backRightStrip;
    private final LEDStrip backLeftStrip;

    private final LEDStrip fullLeftStrip;
    private final LEDStrip fullRightStrip;

    private final LEDStrip fullSideStrips;

    private final LEDStrip backMirroredStrip;

    private final Notifier loadingNotifier;

    public Leds() {
        System.out.println("[Init Leds] Instantiating Leds");
        // if(RobotType.getMode() == Mode.REAL) {
            var addressableStrip = new AddressableStrip(0, 57);
            hardwareStrip = addressableStrip;
        // } else {
        //     hardwareStrip = new HardwareStrip() {
        //         @Override
        //         public int getLength() {
        //             return 0;
        //         }
        //         @Override
        //         public void setLED(int ledIndex, Color color) {}
        //         @Override
        //         public void refresh() {}
        //     };
        // }

        rightStrip = hardwareStrip.substrip(0, 19);
        backStrip = hardwareStrip.substrip(19, 38);
        leftStrip = hardwareStrip.substrip(38, 57).reverse();

        sideStrips = leftStrip.parallel(rightStrip);
        sideStripTips = sideStrips.substrip(15).concat(backStrip.substrip(5, 13));
        dsConnectedStrip = sideStrips.substrip(0, 2);
        lAprilConnectedStrip = sideStrips.substrip(2, 3);
        rAprilConnectedStrip = sideStrips.substrip(3, 4);
        nVisionConnectedStrip = sideStrips.substrip(4, 5);
        
        backRightStrip = backStrip.substrip(0, 10);
        backLeftStrip = backStrip.substrip(9).reverse();
        
        fullLeftStrip = leftStrip.concat(backLeftStrip);
        fullRightStrip = rightStrip.concat(backRightStrip);
        
        fullSideStrips = fullLeftStrip.parallel(fullRightStrip);
        
        backMirroredStrip = backRightStrip.reverse().parallel(backLeftStrip.reverse());

        loadingNotifier = new Notifier(() -> {
            synchronized(this) {
                hardwareStrip.apply(
                    InterpolationStyle.Linear.gradient(
                        Color.kBlack,
                        Color.kDimGray
                    )
                    .apply(
                        TilingFunction.Sinusoidal.tile(
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

    public final AnimationFlag estopped = new AnimationFlag();
    public final AnimationFlag autonomousOverrun = new AnimationFlag();
    public final AnimationFlag noteAcquired = new AnimationFlag();
    public final AnimationFlag noteSecured = new AnimationFlag();
    public final AnimationFlag visionAcquired = new AnimationFlag();
    public final AnimationFlag visionLocked = new AnimationFlag();
    public final AnimationFlag humanPlayerFlash = new AnimationFlag();
    public final AnimationFlag shooterBarGraph = new AnimationFlag();
    public final AnimationFlag defenseSpin = new AnimationFlag();
    public final AnimationFlag climbingMode = new AnimationFlag();
    public final AnimationFlag climbing = new AnimationFlag();
    public boolean lAprilConnected;
    public boolean rAprilConnected;
    public boolean nVisionConnected;
    public boolean shooterReady;
    public double shooterTarget;
    public double shooterSpeed;
    public double climberPos;

    private int skippedFrames = 0;
    private static final int frameSkipAmount = 15;

    @Override
    public synchronized void periodic() {
        if(skippedFrames < frameSkipAmount) {
            skippedFrames++;
            return;
        }
        loadingNotifier.stop();

        // Default alliance color scrolling
        fullSideStrips.apply((pos) -> 
            InterpolationStyle.Linear.gradient(() -> 
                new Color[]{
                    (DriverStation.getAlliance().isEmpty() ? Color.kRed : Color.kBlack),
                    (DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? Color.kRed : Color.kFirstBlue)
                }
            )
            .apply(
                TilingFunction.Sinusoidal.tile(
                    pos*4 - Timer.getFPGATimestamp()
                )
            )
        );

        if(DriverStation.isDisabled()) {
            dsConnectedStrip.apply(DriverStation.isDSAttached() ? Color.kGreen : Color.kOrange);
            lAprilConnectedStrip.apply(lAprilConnected ? Color.kGreen : Color.kOrange);
            rAprilConnectedStrip.apply(rAprilConnected ? Color.kGreen : Color.kOrange);
            nVisionConnectedStrip.apply(nVisionConnected ? Color.kGreen : Color.kOrange);
        }

        if(visionAcquired.get()) {
            sideStripTips.apply(Color.kOrange);
        }
        if(visionLocked.get()) {
            sideStripTips.apply(Color.kPurple);
        }
        if(noteSecured.get()) {
            sideStripTips.apply(Color.kGreen);
        }

        if(visionLocked.get() && DriverStation.isAutonomousEnabled()) {
            sideStripTips.apply(Color.kPurple);
        }

        if(defenseSpin.get()) {
            fullSideStrips.apply(
                InterpolationStyle.Linear.gradient(Color.kBlack, Color.kYellow)
                .apply(
                    TilingFunction.Sinusoidal.tile(
                        Timer.getFPGATimestamp()*4
                    )
                )
            );
        }

        if(humanPlayerFlash.get()) {
            fullSideStrips.apply(
                InterpolationStyle.Step.gradient(Color.kBlack, Color.kWhite)
                .apply(
                    TilingFunction.Sawtooth.tile(
                        Timer.getFPGATimestamp()*8
                    )
                )
            );
        }

        if(noteAcquired.get()) {
            fullSideStrips.apply(
                InterpolationStyle.Linear.gradient(Color.kBlack, Color.kGreen)
                .apply(
                    TilingFunction.Sawtooth.tile(
                        Timer.getFPGATimestamp()*8
                    )
                )
            );
        }

        if(autonomousOverrun.get()) {
            if(!DriverStation.isFMSAttached()) {
                hardwareStrip.apply(
                    InterpolationStyle.Step.gradient(
                        Color.kRed,
                        Color.kBlack
                    )
                    .apply(
                        TilingFunction.Modulo.tile(
                            GameState.getInstance().LAST_ENABLE.getTimeSince()
                        )
                    )
                );
            }
        } else if(
            GameState.getInstance().lastEnabledMode.isAutonomous() && 
            GameState.getInstance().AUTONOMOUS_COMMAND_FINISH.isSet() && 
            !GameState.getInstance().BEGIN_ENABLE.hasBeenSince(15.3)
        ) {
            sideStrips.apply((pos) -> {
                var timeLeft = AutoConstants.allottedAutoTime - (GameState.getInstance().AUTONOMOUS_COMMAND_FINISH.getTimeSince() - GameState.getInstance().BEGIN_ENABLE.getTimeSince());
                var a = GameState.getInstance().AUTONOMOUS_COMMAND_FINISH.getTimeSince() / timeLeft;
                var barPos = 1-a;
                return (pos <= barPos ? Color.kGreen : Color.kBlack);
            });
        }

        if(estopped.get() || DriverStation.isEStopped()) {
            hardwareStrip.apply(Color.kRed);
        }

        //TODO: End game notification
        hardwareStrip.refresh();
    }

    public static class AnimationFlag {
        private boolean scheduled;
        public boolean get() {
            return scheduled;
        }
        public void set(boolean scheduled) {
            this.scheduled = scheduled;
        }

        public Command setCommand() {
            return Commands.startEnd(
                () -> set(true),
                () -> set(false)
            )
            .until(() -> !scheduled);
        }
    }
}
