package frc.robot.subsystems.vision.bucket;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.vision.bucket.BucketCameraIO.BucketCameraTarget;
import frc.util.LazyOptional;
import frc.util.LoggedTunableMeasure;
import frc.util.LoggedTunableNumber;
import frc.util.VirtualSubsystem;
import frc.util.loggerUtil.LoggerUtil;
import frc.util.robotStructure.CameraMount;

public class BucketVision extends VirtualSubsystem {
    private final BucketCamera[] cameras;

    private static final LoggedTunableMeasure<DistanceUnit> updateDistanceThreshold = new LoggedTunableMeasure<>("Vision/Bucket/Updating/Update Distance Threshold", Meters.of(5), Inches);
    private static final LoggedTunableNumber posUpdatingFilteringFactor = new LoggedTunableNumber("Vision/Bucket/Updating/Pos Updating Filtering Factor", 0.8);
    private static final LoggedTunableNumber confUpdatingFilteringFactor = new LoggedTunableNumber("Vision/Bucket/Confidence/Updating Filtering Factor", 0.5);
    private static final LoggedTunableNumber confidencePerAreaPercent = new LoggedTunableNumber("Vision/Bucket/Confidence/Per Area Percent", 1);
    private static final LoggedTunableNumber confidenceDecayPerSecond = new LoggedTunableNumber("Vision/Bucket/Confidence/Decay Per Second", 3);
    private static final LoggedTunableNumber priorityPerConfidence = new LoggedTunableNumber("Vision/Bucket/Priority/Priority Per Confidence", 4);
    private static final LoggedTunableNumber priorityPerDistance = new LoggedTunableNumber("Vision/Bucket/Priority/Priority Per Distance", -2);
    private static final LoggedTunableNumber acquireConfidenceThreshold = new LoggedTunableNumber("Vision/Bucket/Target Threshold/Acquire", -2);
    private static final LoggedTunableNumber detargetConfidenceThreshold = new LoggedTunableNumber("Vision/Bucket/Target Threshold/Detarget", -3);

    private final ArrayList<TrackedBucket> bucketMemories = new ArrayList<>(3);

    private Optional<TrackedBucket> optIntakeTarget = Optional.empty();
    private boolean intakeTargetLocked = false;

    public BucketVision(BucketCamera... cameras) {
        System.out.println("[Init BucketVision] Instantiating BucketVision");
        this.cameras = cameras;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Bucket Cam Poses", Arrays.stream(
            new BucketVisionConstants.BucketCameraConstants[]{
                BucketVisionConstants.bucketCamera
            })
            .map((constants) -> constants.mount.getFieldRelative())
            .toArray(Pose3d[]::new)
        );
        var frameTargets = Arrays
            .stream(cameras)
            .map(BucketCamera::periodic)
            .filter(Optional::isPresent)
            .map(Optional::get)
            .flatMap((result) ->
                Arrays.stream(result.targets).map((target) -> TrackedBucket.from(result.camMeta.mount, target))
            )
            .filter(Optional::isPresent)
            .map(Optional::get)
            .toList()
        ;
        var connections = new ArrayList<TargetMemoryConnection>(bucketMemories.size() * frameTargets.size());
        bucketMemories.forEach(
            (memory) -> frameTargets.forEach(
                (target) -> {
                    if(memory.fieldPos.getDistance(target.fieldPos) < updateDistanceThreshold.in(Meters)) {
                        connections.add(new TargetMemoryConnection(memory, target));
                    }
                }
            )
        );
        connections.sort((a, b) -> Double.compare(a.getDistance(), b.getDistance()));
        var unusedMemories = new ArrayList<>(bucketMemories);
        var unusedTargets = new ArrayList<>(frameTargets);
        while(!connections.isEmpty()) {
            var confirmedConnection = connections.get(0);
            confirmedConnection.memory.updatePosWithFiltering(confirmedConnection.cameraTarget);
            confirmedConnection.memory.updateConfidence();
            unusedMemories.remove(confirmedConnection.memory);
            unusedTargets.remove(confirmedConnection.cameraTarget);
            connections.removeIf((connection) -> 
                connection.memory == confirmedConnection.memory
                || connection.cameraTarget == confirmedConnection.cameraTarget
            );
        }
        unusedMemories.forEach((memory) -> {
            if(RobotState.getInstance().getPose().getTranslation().getDistance(memory.fieldPos) > RobotConstants.robotLength.in(Meters)*0.5) {
                memory.decayConfidence(1);
            }
        });
        unusedTargets.forEach(bucketMemories::add);
        bucketMemories.removeIf((memory) -> memory.confidence <= 0);
        bucketMemories.removeIf((memory) -> Double.isNaN(memory.fieldPos.getX()) || Double.isNaN(memory.fieldPos.getY()));
        bucketMemories.removeIf((memory) -> RobotState.getInstance().getPose().getTranslation().getDistance(memory.fieldPos) <= 0.07);

        if(
            optIntakeTarget.isPresent()
            && (
                optIntakeTarget.get().confidence < detargetConfidenceThreshold.get()
                || !bucketMemories.contains(optIntakeTarget.get())
            )
        ) {
            optIntakeTarget = Optional.empty();
        }
        if(optIntakeTarget.isEmpty() || !intakeTargetLocked) {
            optIntakeTarget = bucketMemories
                .stream()
                .filter((target) -> target.getPriority() >= acquireConfidenceThreshold.get())
                .sorted((a,b) -> Double.compare(b.getPriority(), a.getPriority()))
                .findFirst()
            ;
        }
        
        Leds.getInstance().visionAcquired.setFlag(hasTarget());
        Leds.getInstance().visionLocked.setFlag(targetLocked());
        
        Logger.recordOutput("Vision/Bucket/Bucket Memories", bucketMemories.stream().map(TrackedBucket::toASPose).toArray(Pose3d[]::new));
        Logger.recordOutput("Vision/Bucket/Bucket Confidence", bucketMemories.stream().mapToDouble((note) -> note.confidence).toArray());
        Logger.recordOutput("Vision/Bucket/Bucket Priority", bucketMemories.stream().mapToDouble(TrackedBucket::getPriority).toArray());
        Logger.recordOutput("Vision/Bucket/Target", LoggerUtil.toArray(optIntakeTarget.map(TrackedBucket::toASPose), Pose3d[]::new));
        Logger.recordOutput("Vision/Bucket/Locked Target", LoggerUtil.toArray(optIntakeTarget.filter((a) -> intakeTargetLocked).map(TrackedBucket::toASPose).map(Pose3d::getTranslation), Translation3d[]::new));
    }

    public DoubleSupplier applyDotProduct(Supplier<ChassisSpeeds> joystickFieldRelative) {
        return () -> optIntakeTarget.map((target) -> {
            var robotTrans = RobotState.getInstance().getPose().getTranslation();
            var targetRelRobot = target.fieldPos.minus(robotTrans);
            var targetRelRobotNormalized = targetRelRobot.div(targetRelRobot.getNorm());
            var joystickSpeed = joystickFieldRelative.get();
            var joy = new Translation2d(joystickSpeed.vxMetersPerSecond, joystickSpeed.vyMetersPerSecond);
            var throttle = targetRelRobotNormalized.toVector().dot(joy.toVector());
            return throttle;
        }).orElse(0.0);
    }

    public LazyOptional<ChassisSpeeds> getAutoIntakeTransSpeed(DoubleSupplier throttleSupplier) {
        return () -> optIntakeTarget.map((target) -> {
            var robotTrans = RobotState.getInstance().getPose().getTranslation();
            var targetRelRobot = target.fieldPos.minus(robotTrans);
            var targetRelRobotNormalized = targetRelRobot.div(targetRelRobot.getNorm());
            var finalTrans = targetRelRobotNormalized.times(throttleSupplier.getAsDouble());
            return new ChassisSpeeds(finalTrans.getX(), finalTrans.getY(), 0);
        });
    }

    public LazyOptional<Translation2d> autoIntakeTargetLocation() {
        return () -> optIntakeTarget.map((target) -> target.fieldPos);
    }

    public boolean hasTarget() {
        return optIntakeTarget.isPresent();
    }

    public boolean targetLocked() {
        return intakeTargetLocked;
    }

    public void clearMemory() {
        bucketMemories.clear();
        optIntakeTarget = Optional.empty();
    }

    public Command autoIntake(DoubleSupplier throttle, BooleanSupplier noBucket, Drive drive) {
        return 
            Commands.runOnce(() -> intakeTargetLocked = true)
            .alongWith(
                drive.translationSubsystem.fieldRelative(getAutoIntakeTransSpeed(throttle).orElseGet(ChassisSpeeds::new)),
                drive.rotationalSubsystem.pointTo(autoIntakeTargetLocation(), () -> RobotConstants.intakeForward)
            )
            .onlyWhile(() -> noBucket.getAsBoolean() && optIntakeTarget.isPresent())
            .finallyDo(() -> intakeTargetLocked = false)
            .withName("Auto Intake")
        ;
    }

    private static record TargetMemoryConnection(TrackedBucket memory, TrackedBucket cameraTarget) {
        public double getDistance() {
            return memory.fieldPos.getDistance(cameraTarget.fieldPos);
        }
    }

    public static class TrackedBucket implements StructSerializable {
        public Translation2d fieldPos;
        public double confidence;

        public TrackedBucket(Translation2d fieldPos, double confidence) {
            this.fieldPos = fieldPos;
            this.confidence = confidence * confUpdatingFilteringFactor.get();
        }

        public static Optional<TrackedBucket> from(CameraMount mount, BucketCameraTarget target) {
            var camTransform = mount.getRobotRelative();
            // var targetCamViewTransform = camTransform.plus(
            //     new Transform3d(
            //         Translation3d.kZero,
            //         target.cameraPose
            //     )
            // );
            // var distOut = targetCamViewTransform.getTranslation().getZ() / Math.tan(targetCamViewTransform.getRotation().getY());
            // var distOff = distOut * Math.tan(targetCamViewTransform.getRotation().getZ());
            // var camToTargetTranslation = new Translation3d(distOut, distOff, -camTransform.getZ());
            // var fieldPos = new Pose3d(RobotState.getInstance().getPose())
            //     .transformBy(camTransform)
            //     .transformBy(new Transform3d(camToTargetTranslation, Rotation3d.kZero))
            //     .toPose2d()
            //     .getTranslation()
            // ;
            var cameraToTargetVectorRobotRelative = target.cameraToTargetVector.rotateBy(camTransform.getRotation());
            if (cameraToTargetVectorRobotRelative.getZ() >= 0) {
                // Target above horizon, ignore
                return Optional.empty();
            }
            var toFloorScalingFactor = camTransform.getTranslation().getZ() / -cameraToTargetVectorRobotRelative.getZ();
            var robotToTarget = cameraToTargetVectorRobotRelative.times(toFloorScalingFactor).plus(camTransform.getTranslation());
            var fieldPose = RobotState.getInstance().getPose().transformBy(new Transform2d(robotToTarget.toTranslation2d(), Rotation2d.kZero));

            var confidence = Math.sqrt(target.area) * confidencePerAreaPercent.get();

            return Optional.of(new TrackedBucket(fieldPose.getTranslation(), confidence));
        }

        public void updateConfidence() {
            confidence += confidence * MathUtil.clamp(1 - confUpdatingFilteringFactor.get(), 0, 1); 
        }

        public void updatePosWithFiltering(TrackedBucket newNote) {
            this.fieldPos = fieldPos.interpolate(newNote.fieldPos, posUpdatingFilteringFactor.get());
            this.confidence = newNote.confidence;
        }

        public void decayConfidence(double rate) {
            this.confidence -= confidenceDecayPerSecond.get() * rate * RobotConstants.rioUpdatePeriodSecs;
        }

        public double getPriority() {
            var pose = RobotState.getInstance().getPose();
            var FORR = fieldPos.minus(pose.getTranslation());
            var rotation = pose.getRotation().minus(RobotConstants.intakeForward);
            return 
                confidence * priorityPerConfidence.get() *
                VecBuilder.fill(rotation.getCos(), rotation.getSin()).dot(FORR.toVector().unit()) + 
                FORR.getNorm() * priorityPerDistance.get()
            ;
        }

        public Pose3d toASPose() {
            return new Pose3d(
                new Translation3d(
                    fieldPos.getMeasureX(),
                    fieldPos.getMeasureY(),
                    FieldConstants.bucketHeight
                ),
                new Rotation3d(
                    Degrees.of(180),
                    Degrees.zero(),
                    Degrees.zero()
                )
            );
        }

        public static final TrackedBucketStruct struct = new TrackedBucketStruct();
        public static class TrackedBucketStruct implements Struct<TrackedBucket> {
            @Override
            public Class<TrackedBucket> getTypeClass() {
                return TrackedBucket.class;
            }

            @Override
            public String getTypeName() {
                return "TrackedNote";
            }

            @Override
            public int getSize() {
                return Translation2d.struct.getSize() * 1 + kSizeDouble * 1;
            }

            @Override
            public String getSchema() {
                return "Translation2d fieldPos;double confidence";
            }

            @Override
            public Struct<?>[] getNested() {
                return new Struct<?>[] {Translation2d.struct};
            }

            @Override
            public TrackedBucket unpack(ByteBuffer bb) {
                var fieldPos = Translation2d.struct.unpack(bb);
                var confidence = bb.getDouble();
                return new TrackedBucket(fieldPos, confidence);
            }

            @Override
            public void pack(ByteBuffer bb, TrackedBucket value) {
                Translation2d.struct.pack(bb, value.fieldPos);
                bb.putDouble(value.confidence);
            }
        }
    }
}
