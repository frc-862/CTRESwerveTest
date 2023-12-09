package frc.robot.subsytems.swerve;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public double drivePositionDeg = 0d;
        public double driveVelocityRPM = 0d;
        public double driveAppliedVolts = 0d;
        public double driveCurrentAmps = 0d;

        public double steerPositionDeg = 0d;
        public double steerVelocityRPM = 0d;
        public double steerAppliedVolts = 0d;
        public double steerCurrentAmps = 0d;

        public SwerveModulePosition position = new SwerveModulePosition();
        public SwerveModuleState state = new SwerveModuleState();
        public BaseStatusSignal[] statusSignals = new BaseStatusSignal[0];
    }

    public default void updateInputs(ModuleIOInputs inputs) {}

    public default void apply(SwerveModuleState state, boolean isOpenLoop) {}

    public default void resetPosition() {}

}
