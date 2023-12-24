package frc.robot.subsytems.swerve;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.signals.MagnetHealthValue;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public class ModuleIOCTRE implements ModuleIO {
    SwerveModuleState lastState;
    boolean runningOpenLoop;

    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder steerEncoder;

    private final StatusSignal<Double> drivePosition;
    private final StatusSignal<Double> driveVelocity;
    private final StatusSignal<Double> driveAppliedVolts;
    private final StatusSignal<Double> driveCurrent;

    private final StatusSignal<Double> steerPosition;
    private final StatusSignal<Double> steerVelocity;
    private final StatusSignal<Double> steerAppliedVolts;
    private final StatusSignal<Double> steerCurrent;

    private final StatusSignal<Double> steerEncoderPosition;
    private final StatusSignal<Double> steerEncoderVelocity;
    private final StatusSignal<MagnetHealthValue> steerEncoderAppliedVolts;


    public ModuleIOCTRE(SwerveModule module) {
        driveMotor = module.getDriveMotor();
        steerMotor = module.getSteerMotor();
        steerEncoder = module.getCANcoder();

        drivePosition = driveMotor.getPosition().clone();
        driveVelocity = driveMotor.getVelocity().clone();
        driveAppliedVolts = driveMotor.getMotorVoltage();
        driveCurrent = driveMotor.getStatorCurrent();

        steerPosition = steerMotor.getPosition().clone();
        steerVelocity = steerMotor.getVelocity().clone();
        steerAppliedVolts = steerMotor.getMotorVoltage();
        steerCurrent = steerMotor.getStatorCurrent();

        steerEncoderPosition = steerEncoder.getPosition().clone();
        steerEncoderVelocity = steerEncoder.getVelocity().clone();
        steerEncoderAppliedVolts = steerEncoder.getMagnetHealth();
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {

        BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent, steerPosition, steerVelocity, steerAppliedVolts, steerCurrent);

        inputs.drivePositionDeg = drivePosition.getValueAsDouble() * 360d;
        inputs.driveVelocityRPM = driveVelocity.getValueAsDouble() / 60d;
        inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
        inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();

        inputs.steerPositionDeg = steerPosition.getValueAsDouble() * 360d;
        inputs.steerVelocityRPM = steerVelocity.getValueAsDouble() / 60d;
        inputs.steerAppliedVolts = steerAppliedVolts.getValueAsDouble();
        inputs.steerCurrentAmps = steerCurrent.getValueAsDouble();

        inputs.steerEncoderPositionDeg = steerEncoderPosition.getValueAsDouble() * 360d;
        inputs.steerEncoderVelocityRPM = steerEncoderVelocity.getValueAsDouble() / 60d;
        inputs.steerEncoderMagnetHealth = steerEncoderAppliedVolts.getValue().toString();

        inputs.state = lastState;

        inputs.runningOpenLoop = runningOpenLoop;
    }
}
