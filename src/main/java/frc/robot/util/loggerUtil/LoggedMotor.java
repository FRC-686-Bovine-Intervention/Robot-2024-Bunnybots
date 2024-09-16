package frc.robot.util.loggerUtil;

import java.nio.ByteBuffer;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class LoggedMotor implements StructSerializable {
    public double positionRad = Double.NaN;
    public double velocityRadPerSec = Double.NaN;
    public double appliedVolts = Double.NaN;
    public double currentAmps = Double.NaN;
    public double tempCelsius = Double.NaN;

    public void updateFrom(TalonFX talon) {
        this.positionRad = Units.rotationsToRadians(talon.getPosition().getValueAsDouble());
        this.velocityRadPerSec = talon.getVelocity().getValueAsDouble();
        this.appliedVolts = talon.getMotorVoltage().getValueAsDouble();
        this.currentAmps = talon.getStatorCurrent().getValueAsDouble();
        this.tempCelsius = talon.getDeviceTemp().getValueAsDouble();
    }

    public void updateFrom(CANSparkMax spark) {
        this.positionRad = Units.rotationsToRadians(spark.getEncoder().getPosition());
        this.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(spark.getEncoder().getVelocity());
        this.appliedVolts = spark.getAppliedOutput() * 12;
        this.currentAmps = spark.getOutputCurrent();
    }

    public void updateFrom(DCMotorSim sim) {
        this.positionRad = sim.getAngularPositionRad();
        this.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
        this.currentAmps = sim.getCurrentDrawAmps();
    }
    public void updateFrom(DCMotorSim sim, double appliedVolts) {
        updateFrom(sim);
        this.appliedVolts = appliedVolts;
    }

    public void updateFrom(FlywheelSim sim) {
        this.positionRad += sim.getAngularVelocityRadPerSec() * Constants.dtSeconds;
        this.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
        this.currentAmps = sim.getCurrentDrawAmps();
    }
    public void updateFrom(FlywheelSim sim, double appliedVolts) {
        updateFrom(sim);
        this.appliedVolts = appliedVolts;
    }

    public void updateFrom(SingleJointedArmSim sim) {
        this.positionRad = sim.getAngleRads();
        this.velocityRadPerSec = sim.getVelocityRadPerSec();
        this.currentAmps = sim.getCurrentDrawAmps();
    }
    public void updateFrom(SingleJointedArmSim sim, double appliedVolts) {
        updateFrom(sim);
        this.appliedVolts = appliedVolts;
    }

    public static final LoggedMotorStruct struct = new LoggedMotorStruct();

    public static class LoggedMotorStruct implements Struct<LoggedMotor> {
        @Override
        public Class<LoggedMotor> getTypeClass() {
            return LoggedMotor.class;
        }

        @Override
        public String getTypeString() {
            return "struct:Motor";
        }

        @Override
        public int getSize() {
            return kSizeDouble * 5;
        }

        @Override
        public String getSchema() {
            return "double PositionRad;double VelocityRadPerSec;double AppliedVolts;double CurrentAmps;double TempCelsius";
        }

        @Override
        public LoggedMotor unpack(ByteBuffer bb) {
            var motor = new LoggedMotor();
            motor.positionRad = bb.getDouble();
            motor.velocityRadPerSec = bb.getDouble();
            motor.appliedVolts = bb.getDouble();
            motor.currentAmps = bb.getDouble();
            motor.tempCelsius = bb.getDouble();
            return motor;
        }

        @Override
        public void pack(ByteBuffer bb, LoggedMotor value) {
            bb.putDouble(value.positionRad);
            bb.putDouble(value.velocityRadPerSec);
            bb.putDouble(value.appliedVolts);
            bb.putDouble(value.currentAmps);
            bb.putDouble(value.tempCelsius);
        }
    }
}
