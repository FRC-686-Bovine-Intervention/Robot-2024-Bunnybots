package frc.util.loggerUtil.inputs;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.nio.ByteBuffer;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Robot;

public class LoggedEncoder implements StructSerializable {
    public final MutAngle position = Radians.mutable(0);
    public final MutAngularVelocity velocity = RadiansPerSecond.mutable(0);

    public void updateFrom(TalonFX talon) {
        this.position.mut_replace(talon.getPosition().getValue());
        this.velocity.mut_replace(talon.getVelocity().getValue());
    }
    public void updateFrom(CANcoder canCoder) {
        this.position.mut_replace(canCoder.getPosition().getValue());
        this.velocity.mut_replace(canCoder.getVelocity().getValue());
    }

    public void updateFrom(AbsoluteEncoder encoder) {
        this.position.mut_replace(encoder.getPosition(), Rotations);
        this.velocity.mut_replace(encoder.getVelocity(), RotationsPerSecond);
    }

    public void updateFrom(DCMotorSim sim) {
        this.position.mut_replace(sim.getAngularPositionRad(), Radians);
        this.velocity.mut_replace(sim.getAngularVelocityRadPerSec(), RadiansPerSecond);
    }

    public void updateFrom(FlywheelSim sim) {
        this.position.mut_acc(sim.getAngularVelocityRadPerSec() * Robot.defaultPeriodSecs);
        this.velocity.mut_replace(sim.getAngularVelocityRadPerSec(), RadiansPerSecond);
    }

    public void updateFrom(SingleJointedArmSim sim) {
        this.position.mut_replace(sim.getAngleRads(), Radians);
        this.velocity.mut_replace(sim.getVelocityRadPerSec(), RadiansPerSecond);
    }

    public static final LoggedEncoderStruct struct = new LoggedEncoderStruct();
    // public static final LoggedEncoderProto proto = new LoggedEncoderProto();
    
    public static class LoggedEncoderStruct implements Struct<LoggedEncoder> {
        @Override
        public Class<LoggedEncoder> getTypeClass() {
            return LoggedEncoder.class;
        }

        @Override
        public String getTypeName() {
            return "Encoder";
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
            encoder.position.mut_setBaseUnitMagnitude(bb.getDouble());
            encoder.velocity.mut_setBaseUnitMagnitude(bb.getDouble());
            return encoder;
        }

        @Override
        public void unpackInto(LoggedEncoder out, ByteBuffer bb) {
            out.position.mut_setBaseUnitMagnitude(bb.getDouble());
            out.velocity.mut_setBaseUnitMagnitude(bb.getDouble());
        }

        @Override
        public void pack(ByteBuffer bb, LoggedEncoder value) {
            bb.putDouble(value.position.baseUnitMagnitude());
            bb.putDouble(value.velocity.baseUnitMagnitude());
        }
    }

    // public static class LoggedEncoderProto implements Protobuf<LoggedEncoder, ProtobufLoggedEncoder> {
    //     @Override
    //     public Class<LoggedEncoder> getTypeClass() {
    //         return LoggedEncoder.class;
    //     }

    //     @Override
    //     public Descriptor getDescriptor() {
    //         // TODO Auto-generated method stub
    //         throw new UnsupportedOperationException("Unimplemented method 'getDescriptor'");
    //     }

    //     @Override
    //     public ProtobufLoggedEncoder createMessage() {
    //         return ProtobufLoggedEncoder.newInstance();
    //     }

    //     @Override
    //     public LoggedEncoder unpack(ProtobufLoggedEncoder msg) {
    //         var obj = new LoggedEncoder();
    //         obj.position.mut_setBaseUnitMagnitude(msg.getPosition());
    //         obj.velocity.mut_setBaseUnitMagnitude(msg.getVelocity());
    //         return obj;
    //     }

    //     @Override
    //     public void pack(ProtobufLoggedEncoder msg, LoggedEncoder value) {
    //         msg.setPosition(value.position.baseUnitMagnitude());
    //         msg.setVelocity(value.velocity.baseUnitMagnitude());
    //     }

    //     public static class ProtobufLoggedEncoder extends ProtoMessage<ProtobufLoggedEncoder> implements Cloneable {
    //         private static final long serialVersionUID = 0L;

    //         private double position;
    //         private double velocity;

    //         private ProtobufLoggedEncoder() {}
    //         public static ProtobufLoggedEncoder newInstance() {
    //             return new ProtobufLoggedEncoder();
    //         }

    //         public boolean hasPosition() {
    //             return (bitField0_ & 0x00000001) != 0;
    //         }
    //         public ProtobufLoggedEncoder clearPosition() {
    //             bitField0_ &= ~0x00000001;
    //             position = 0D;
    //             return this;
    //         }
    //         public double getPosition() {
    //             return position;
    //         }
    //         public ProtobufLoggedEncoder setPosition(final double position) {
    //             bitField0_ |= 0x00000001;
    //             this.position = position;
    //             return this;
    //         }

    //         public boolean hasVelocity() {
    //             return (bitField0_ & 0x00000002) != 0;
    //         }
    //         public ProtobufLoggedEncoder clearVelocity() {
    //             bitField0_ &= ~0x00000001;
    //             velocity = 0D;
    //             return this;
    //         }
    //         public double getVelocity() {
    //             return velocity;
    //         }
    //         public ProtobufLoggedEncoder setVelocity(final double velocity) {
    //             bitField0_ |= 0x00000002;
    //             this.velocity = velocity;
    //             return this;
    //         }

    //         @Override
    //         public boolean isEmpty() {
    //             return bitField0_ == 0;
    //         }
            
    //         @Override
    //         public ProtobufLoggedEncoder copyFrom(final ProtobufLoggedEncoder other) {
    //             cachedSize = other.cachedSize;
    //             if((bitField0_ | other.bitField0_) != 0) {
    //                 bitField0_ = other.bitField0_;
    //                 position = other.position;
    //                 velocity = other.velocity;
    //             }
    //             return this;
    //         }

    //         @Override
    //         public ProtobufLoggedEncoder mergeFrom(final ProtobufLoggedEncoder other) {
    //             if(other.isEmpty()) {
    //                 return this;
    //             }
    //             cachedSize = -1;
    //             if(other.hasPosition()) {
    //                 setPosition(other.position);
    //             }
    //             if(other.hasVelocity()) {
    //                 setVelocity(other.velocity);
    //             }
    //             return this;
    //         }

    //         @Override
    //         public ProtobufLoggedEncoder clear() {
    //             if(isEmpty()) {
    //                 return this;
    //             }
    //             cachedSize = -1;
    //             bitField0_ = 0;
    //             position = 0D;
    //             velocity = 0D;
    //             return this;
    //         }
    //         @Override
    //         public ProtobufLoggedEncoder clearQuick() {
    //             if(isEmpty()) {
    //                 return this;
    //             }
    //             cachedSize = -1;
    //             bitField0_ = 0;
    //             return this;
    //         }
            
    //         @Override
    //         public boolean equals(Object obj) {
    //             if (obj == this) {
    //                 return true;
    //             }
    //             if (!(obj instanceof ProtobufLoggedEncoder)) {
    //                 return false;
    //             }
    //             ProtobufLoggedEncoder other = (ProtobufLoggedEncoder) obj;
    //             return bitField0_ == other.bitField0_
    //                 && (!hasPosition() || ProtoUtil.isEqual(position, other.position))
    //                 && (!hasVelocity() || ProtoUtil.isEqual(velocity, other.velocity));
    //         }

    //         @Override
    //         public void writeTo(ProtoSink output) throws IOException {
    //             if((bitField0_ & 0x00000001) != 0) {
    //                 output.writeRawByte((byte) 9);
    //                 output.writeDoubleNoTag(position);
    //             }
    //             if((bitField0_ & 0x00000002) != 0) {
    //                 output.writeRawByte((byte) 17);
    //                 output.writeDoubleNoTag(velocity);
    //             }
    //         }

    //         @Override
    //         protected int computeSerializedSize() {
    //             int size = 0;
    //             if ((bitField0_ & 0x00000001) != 0) {
    //                 size += 9;
    //             }
    //             if ((bitField0_ & 0x00000002) != 0) {
    //                 size += 9;
    //             }
    //             return size;
    //         }

    //         @Override
    //         public ProtobufLoggedEncoder mergeFrom(ProtoSource input) throws IOException {
    //             int tag = input.readTag();
    //             while (true) {
    //                 switch (tag) {
    //                     case 9:
    //                         position = input.readDouble();
    //                         bitField0_ |= 0x00000001;
    //                         tag = input.readTag();
    //                         if(tag != 17) {
    //                             break;
    //                         }
    //                     case 17:
    //                         position = input.readDouble();
    //                         bitField0_ |= 0x00000002;
    //                         tag = input.readTag();
    //                         if(tag != 0) {
    //                             break;
    //                         }
    //                     case 0:
    //                         return this;
    //                     default:
    //                         if(!input.skipField(tag)) {
    //                             return this;
    //                         }
    //                         tag = input.readTag();
    //                         break;
    //                 }
    //             }
    //         }

    //         @Override
    //         public ProtobufLoggedEncoder clone() {
    //             return new ProtobufLoggedEncoder().copyFrom(this);
    //         }
    //     }
    // }
}
