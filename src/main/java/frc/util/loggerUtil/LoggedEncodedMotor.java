package frc.util.loggerUtil;

import java.nio.ByteBuffer;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class LoggedEncodedMotor implements StructSerializable {
    public final LoggedEncoder encoder;
    public final LoggedMotor motor;

    public LoggedEncodedMotor() {
        this(new LoggedEncoder(), new LoggedMotor());
    }
    public LoggedEncodedMotor(LoggedEncoder encoder, LoggedMotor motor) {
        this.encoder = encoder;
        this.motor = motor;
    }

    public void updateFrom(TalonFX talon) {
        encoder.updateFrom(talon);
        motor.updateFrom(talon);
    }

    public void updateFrom(CANSparkMax spark) {
        encoder.updateFrom(spark.getAbsoluteEncoder());
        motor.updateFrom(spark);
    }

    public void updateFrom(DCMotorSim sim) {
        encoder.updateFrom(sim);
        motor.updateFrom(sim);
    }
    public void updateFrom(DCMotorSim sim, Voltage appliedVolts) {
        encoder.updateFrom(sim);
        motor.updateFrom(sim, appliedVolts);
    }

    public void updateFrom(FlywheelSim sim) {
        encoder.updateFrom(sim);
        motor.updateFrom(sim);
    }
    public void updateFrom(FlywheelSim sim, Voltage appliedVolts) {
        encoder.updateFrom(sim);
        motor.updateFrom(sim, appliedVolts);
    }

    public void updateFrom(SingleJointedArmSim sim) {
        encoder.updateFrom(sim);
        motor.updateFrom(sim);
    }
    public void updateFrom(SingleJointedArmSim sim, Voltage appliedVolts) {
        encoder.updateFrom(sim);
        motor.updateFrom(sim, appliedVolts);
    }

    public static final LoggedEncodedMotorStruct struct = new LoggedEncodedMotorStruct();

    public static class LoggedEncodedMotorStruct implements Struct<LoggedEncodedMotor> {
        @Override
        public Class<LoggedEncodedMotor> getTypeClass() {
            return LoggedEncodedMotor.class;
        }

        @Override
        public String getTypeName() {
            return "EncodedMotor";
        }
        
        @Override
        public Struct<?>[] getNested() {
            return new Struct<?>[] {LoggedEncoder.struct, LoggedMotor.struct};
        }

        @Override
        public int getSize() {
            return LoggedEncoder.struct.getSize() * 1 + LoggedMotor.struct.getSize() * 1;
        }

        @Override
        public String getSchema() {
            return "Encoder encoder; Motor motor";
        }

        @Override
        public LoggedEncodedMotor unpack(ByteBuffer bb) {
            var encoder = LoggedEncoder.struct.unpack(bb);
            var motor = LoggedMotor.struct.unpack(bb);
            return new LoggedEncodedMotor(encoder, motor);
        }

        @Override
        public void pack(ByteBuffer bb, LoggedEncodedMotor value) {
            LoggedEncoder.struct.pack(bb, value.encoder);
            LoggedMotor.struct.pack(bb, value.motor);
        }
    }
}
