package frc.robot.subsytems.collector;

import org.littletonrobotics.junction.AutoLog;

public interface CollectorIO {
    @AutoLog
    public static class CollectorIOInputs {
        public double positionDeg = 0d;
        public double velocityRPM = 0d;
        public double appliedVolts = 0d;
        public double currentAmps = 0d;
    }

    public default void updateInputs(CollectorIOInputs inputs) {}

    public default void setVoltage(double volts) {}

    public default void setPower(double power) {}

    public default void stop() {}
}
