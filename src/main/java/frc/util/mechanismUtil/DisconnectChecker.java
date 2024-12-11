package frc.util.mechanismUtil;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import frc.util.loggerUtil.inputs.LoggedEncoder;

public class DisconnectChecker {
    private final GearRatio gearRatio;
    private final Angle positionTolerance;
    private final AngularVelocity velocityTolerance;
    private final Timer positionDisconnectTime = new Timer();

    public DisconnectChecker(GearRatio gearRatio, Angle positionTolerance, AngularVelocity velocityTolerance) {
        this.gearRatio = gearRatio;
        this.positionTolerance = positionTolerance;
        this.velocityTolerance = velocityTolerance;
    }

    public void update(LoggedEncoder encoder1, LoggedEncoder encoder2) {
        update(encoder1.position, encoder1.velocity, encoder2.position, encoder2.velocity);
    }
    public void update(Angle position1, AngularVelocity velocity1, Angle position2, AngularVelocity velocity2) {
        
    }
}
