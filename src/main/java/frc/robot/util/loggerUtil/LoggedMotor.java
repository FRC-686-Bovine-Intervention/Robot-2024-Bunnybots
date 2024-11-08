package frc.robot.util.loggerUtil;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Volts;

import java.nio.ByteBuffer;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutTemperature;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class LoggedMotor implements StructSerializable {
    public final MutVoltage appliedVoltage = Volts.mutable(0);
    public final MutCurrent current = Amps.mutable(0);
    public final MutTemperature temperature = Celsius.mutable(0);

    public void updateFrom(TalonFX talon) {
        this.appliedVoltage.mut_replace(talon.getMotorVoltage().getValueAsDouble(), Volts);
        this.current.mut_replace(talon.getStatorCurrent().getValueAsDouble(), Amps);
        this.temperature.mut_replace(talon.getDeviceTemp().getValueAsDouble(), Celsius);
    }

    public void updateFrom(CANSparkMax spark) {
        this.appliedVoltage.mut_replace(spark.getAppliedOutput() * 12, Volts);
        this.current.mut_replace(spark.getOutputCurrent(), Amps);
    }

    public void updateFrom(DCMotorSim sim) {
        this.current.mut_replace(sim.getCurrentDrawAmps(), Amps);
    }
    public void updateFrom(DCMotorSim sim, Voltage appliedVolts) {
        updateFrom(sim);
        this.appliedVoltage.mut_replace(appliedVolts);
    }

    public void updateFrom(FlywheelSim sim) {
        this.current.mut_replace(sim.getCurrentDrawAmps(), Amps);
    }
    public void updateFrom(FlywheelSim sim, Voltage appliedVolts) {
        updateFrom(sim);
        this.appliedVoltage.mut_replace(appliedVolts);
    }

    public void updateFrom(SingleJointedArmSim sim) {
        this.current.mut_replace(sim.getCurrentDrawAmps(), Amps);
    }
    public void updateFrom(SingleJointedArmSim sim, Voltage appliedVolts) {
        updateFrom(sim);
        this.appliedVoltage.mut_replace(appliedVolts);
    }

    public static final LoggedMotorStruct struct = new LoggedMotorStruct();

    public static class LoggedMotorStruct implements Struct<LoggedMotor> {
        @Override
        public Class<LoggedMotor> getTypeClass() {
            return LoggedMotor.class;
        }

        @Override
        public String getTypeName() {
            return "Motor";
        }

        @Override
        public int getSize() {
            return kSizeDouble * 3;
        }

        @Override
        public String getSchema() {
            return "double AppliedVolts;double CurrentAmps;double TempCelsius";
        }

        @Override
        public LoggedMotor unpack(ByteBuffer bb) {
            var motor = new LoggedMotor();
            motor.appliedVoltage.mut_setBaseUnitMagnitude(bb.getDouble());
            motor.current.mut_setBaseUnitMagnitude(bb.getDouble());
            motor.temperature.mut_setBaseUnitMagnitude(bb.getDouble());
            return motor;
        }

        @Override
        public void pack(ByteBuffer bb, LoggedMotor value) {
            bb.putDouble(value.appliedVoltage.baseUnitMagnitude());
            bb.putDouble(value.current.baseUnitMagnitude());
            bb.putDouble(value.temperature.baseUnitMagnitude());
        }
    }
}
