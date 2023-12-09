package frc.robot.subsytems.collector;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.thunder.math.Conversions;

public class CollectorIOTalonFX implements CollectorIO {
    private final double GEAR_REDUCTION = 1.0;

    private final TalonFX motor = new TalonFX(0);

    private final StatusSignal<Double> position = motor.getPosition();
    private final StatusSignal<Double> velocity = motor.getVelocity();
    private final StatusSignal<Double> appliedVolts = motor.getMotorVoltage();
    private final StatusSignal<Double> current = motor.getStatorCurrent();

    public CollectorIOTalonFX() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        //Config things here
        motor.getConfigurator().apply(config);

        BaseStatusSignal.setUpdateFrequencyForAll(250, position, velocity, appliedVolts, current);
        motor.optimizeBusUtilization();

    }

    @Override
    public void updateInputs(CollectorIOInputs inputs) {
        BaseStatusSignal.refreshAll(position, velocity, appliedVolts, current);
        inputs.positionDeg = Conversions.getOutputShaftRotations(position.getValueAsDouble(), GEAR_REDUCTION) * 360.0;
        inputs.velocityRPM = Conversions.getOutputShaftRotations(velocity.getValueAsDouble(), GEAR_REDUCTION) * 60.0;
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.currentAmps = current.getValueAsDouble();
    }

    @Override
    public void setPower(double power) {
        motor.set(power);
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

}
