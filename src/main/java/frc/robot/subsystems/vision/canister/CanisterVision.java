package frc.robot.subsystems.vision.canister;

import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Optional;
import java.util.stream.Collectors;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotState;
import frc.robot.constants.RobotConstants;
import frc.util.LazyOptional;
import frc.util.LoggedInternalButton;
import frc.util.VirtualSubsystem;
import frc.util.loggerUtil.LoggedGroundVisionTarget;

public class CanisterVision extends VirtualSubsystem {
    private final CanisterVisionIO io;
    private final CanisterVisionIOInputsAutoLogged inputs = new CanisterVisionIOInputsAutoLogged();

    private final ArrayList<TrackedCanister> memories = new ArrayList<>();

    private Optional<TrackedCanister> intakeTarget = Optional.empty();
    private LoggedInternalButton intakeTargetLocked = new LoggedInternalButton("Vision/Canister/Intake Target Locked");

    public CanisterVision(CanisterVisionIO io) {
        System.out.println("[Init CanisterVision] Instantiating CanisterVision");
        this.io = io;
        System.out.println("[Init CanisterVision] CanisterVision IO: " + this.io.getClass().getSimpleName());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/CanisterVision", inputs);
        var frameTargets = 
            Arrays.stream(inputs.trackedCanisters)
                .map((tracked) ->  new TrackedCanister(tracked))
                .collect(Collectors.toList());
        var connections = new ArrayList<MemoryConnection>();
        memories.forEach((memory) -> {
            frameTargets.forEach(
                (target) -> {
                    if (
                        memory.fieldPos.getDistance(target.fieldPos) <
                        CanisterVisionConstants.updateDistanceThreshold.get()
                    ) {
                        connections.add(new MemoryConnection(memory, target));
                    }
                }
            );
        });
        connections.sort((a, b) -> (int) Math.signum(a.getDistance() - b.getDistance()));
        var unusedMemories = new ArrayList<>(memories);
        var unusedTargets = new ArrayList<>(frameTargets);
        while(!connections.isEmpty()) {
            var confirmedConnection = connections.get(0);
            confirmedConnection.memory.updatePosWithFiltering(confirmedConnection.target);
            unusedMemories.remove(confirmedConnection.memory);
            unusedTargets.remove(confirmedConnection.target);
            connections.removeIf((connection) -> 
                connection.memory == confirmedConnection.memory ||
                connection.target == confirmedConnection.target
            );
        }
        unusedMemories.forEach((memory) -> {
            if(
                RobotState.getInstance().getPose().getTranslation().getDistance(memory.fieldPos) >
                RobotConstants.robotLength.in(Meters)*0.5
            ) {
                memory.decayConfidence(1);
            }
        });
        unusedTargets.forEach((target) -> memories.add(target));
        memories.removeIf((memory) -> memory.target.confidence <= 0);
        memories.removeIf((memory) -> Double.isNaN(memory.fieldPos.getX()));
        memories.removeIf((memory) ->
            RobotState.getInstance().getPose().getTranslation().getDistance(memory.fieldPos) <= 0.07
        );
        
        if(
            intakeTarget.isPresent() &&
            (
                intakeTarget.get().target.confidence < CanisterVisionConstants.detargetConfidenceThreshold.get() ||
                !memories.contains(intakeTarget.get())
            )
        ) {
            intakeTarget = Optional.empty();
        }

        if(intakeTarget.isEmpty() || !intakeTargetLocked.getAsBoolean()) {
            intakeTarget = memories.stream()
                .filter((target) -> target.getPriority() >= CanisterVisionConstants.acquireConfidenceThreshold.get())
                .sorted((a,b) -> (int)Math.signum(b.getPriority() - a.getPriority()))
                .findFirst();
        }

        Logger.recordOutput("Vision/Canister/Memories",
            memories.stream().map(TrackedCanister::toPose2d).toArray(Pose2d[]::new));
        Logger.recordOutput("Vision/Canister/Confidence",
            memories.stream().mapToDouble((canister) -> canister.target.confidence).toArray());
        Logger.recordOutput("Vision/Canister/Priority",
            memories.stream().mapToDouble(TrackedCanister::getPriority).toArray());
        Logger.recordOutput("Vision/Canister/Target",
            intakeTarget.isPresent() ? intakeTarget.get().toPose2d() : new Pose2d());
        Logger.recordOutput("Vision/Canister/Locked Target",
            intakeTarget.filter((a) -> intakeTargetLocked.getAsBoolean()).isPresent() ?
            intakeTarget.filter((a) -> intakeTargetLocked.getAsBoolean()).get().toPose2d() :
            new Pose2d());
    }
    
    public LazyOptional<Translation2d> autoIntakeTargetLocation() {
        return () -> intakeTarget.map((target) -> target.fieldPos);
    }

    public boolean hasTarget() {
        return intakeTarget.isPresent();
    }

    public boolean targetLocked() {
        return intakeTargetLocked.getAsBoolean();
    }

    public void clearMemory() {
        memories.clear();
        intakeTarget = Optional.empty();
    }

    private static record MemoryConnection(TrackedCanister memory, TrackedCanister target) {
        public double getDistance() {
            return memory.fieldPos.getDistance(target.fieldPos);
        }
    }

    public class TrackedCanister {
        private LoggedGroundVisionTarget target;
        private Translation2d fieldPos;

        public TrackedCanister(LoggedGroundVisionTarget target) {
            this.target = target;
            this.fieldPos = 
                RobotState.getInstance().getPose()
                    .transformBy(new Transform2d(target.robotRelativePos, Rotation2d.kZero))
                    .getTranslation();
        }

        public void decayConfidence(double rate) {
            target.withConfidence(
                target.confidence -
                CanisterVisionConstants.confidenceDecayPerSecond.get() *
                rate * RobotConstants.rioUpdatePeriodSecs
            );
        }

        public void updatePosWithFiltering(TrackedCanister other) {
            fieldPos = 
                fieldPos.interpolate(
                    other.fieldPos,
                    CanisterVisionConstants.posUpdatingFilteringFactor.get()
                );
            target = other.target;
        }

        public double getPriority() {
            var pose = RobotState.getInstance().getPose();
            var FORR = fieldPos.minus(pose.getTranslation());
            var rotation = pose.getRotation().minus(RobotConstants.intakeForward);
            return 
                target.confidence * CanisterVisionConstants.priorityPerConfidence.get() *
                VecBuilder.fill(rotation.getCos(), rotation.getSin()).dot(FORR.toVector().unit()) + 
                FORR.getNorm() * CanisterVisionConstants.priorityPerDistance.get();
        }

        public Pose2d toPose2d() {
            return new Pose2d(fieldPos, Rotation2d.kZero);
        }
    }
}
