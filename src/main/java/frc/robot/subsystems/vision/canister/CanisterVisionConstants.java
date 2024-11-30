package frc.robot.subsystems.vision.canister;

import frc.util.LoggedTunableNumber;

public class CanisterVisionConstants {
    public static final LoggedTunableNumber updateDistanceThreshold = new LoggedTunableNumber("Vision/Note/updateDistanceThreshold", 5);
    public static final LoggedTunableNumber posUpdatingFilteringFactor = new LoggedTunableNumber("Vision/Canister/PosUpdatingFilteringFactor", 0.8);
    public static final LoggedTunableNumber confidencePerAreaPercent = new LoggedTunableNumber("Vision/Canister/Confidence/PerAreaPercent", 1);
    public static final LoggedTunableNumber confidenceDecayPerSecond = new LoggedTunableNumber("Vision/Canister/Confidence/DecayPerSecond", 3);
    public static final LoggedTunableNumber priorityPerConfidence = new LoggedTunableNumber("Vision/Note/Priority/PriorityPerConfidence", 4);
    public static final LoggedTunableNumber priorityPerDistance = new LoggedTunableNumber("Vision/Note/Priority/PriorityPerDistance", -2);
    public static final LoggedTunableNumber acquireConfidenceThreshold = new LoggedTunableNumber("Vision/Note/Target Threshold/Acquire", -2);
    public static final LoggedTunableNumber detargetConfidenceThreshold = new LoggedTunableNumber("Vision/Note/Target Threshold/Detarget", -3);

}
