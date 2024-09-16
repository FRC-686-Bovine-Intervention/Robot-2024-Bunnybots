package frc.robot.util.loggerUtil;

import java.nio.ByteBuffer;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class LoggedEncoder implements StructSerializable {
    public double positionRad = Double.NaN;
    public double velocityRadPerSec = Double.NaN;

    public void updateFrom(TalonFX talon) {
        this.positionRad = Units.rotationsToRadians(talon.getPosition().getValueAsDouble());
        this.velocityRadPerSec = Units.rotationsToRadians(talon.getVelocity().getValueAsDouble());
    }
    public void updateFrom(CANcoder canCoder) {
        this.positionRad = Units.rotationsToRadians(canCoder.getPosition().getValueAsDouble());
        this.velocityRadPerSec = Units.rotationsToRadians(canCoder.getVelocity().getValueAsDouble());
    }

    public void updateFrom(SingleJointedArmSim sim) {
        this.positionRad = sim.getAngleRads();
        this.velocityRadPerSec = sim.getVelocityRadPerSec();
    }

    public static final LoggedEncoderStruct struct = new LoggedEncoderStruct();
    
    public static class LoggedEncoderStruct implements Struct<LoggedEncoder> {
        @Override
        public Class<LoggedEncoder> getTypeClass() {
            return LoggedEncoder.class;
        }

        @Override
        public String getTypeString() {
            return "struct:Encoder";
        }

        @Override
        public int getSize() {
            return kSizeDouble * 2;
        }

        @Override
        public String getSchema() {
            return "double PositionRad;double VelocityRadPerSec";
        }

        @Override
        public LoggedEncoder unpack(ByteBuffer bb) {
            var encoder = new LoggedEncoder();
            encoder.positionRad = bb.getDouble();
            encoder.velocityRadPerSec = bb.getDouble();
            return encoder;
        }

        @Override
        public void pack(ByteBuffer bb, LoggedEncoder value) {
            bb.putDouble(value.positionRad);
            bb.putDouble(value.velocityRadPerSec);
        }
    }
}
