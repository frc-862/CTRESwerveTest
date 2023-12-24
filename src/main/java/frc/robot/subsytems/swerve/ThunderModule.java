package frc.robot.subsytems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.signals.MagnetHealthValue;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public class ThunderModule extends SwerveModule implements ModuleIO {
    SwerveModuleState lastState;
    boolean runningOpenLoop;

    private final TalonFX driveMotor = getDriveMotor();
    private final TalonFX steerMotor = getSteerMotor();
    private final CANcoder steerEncoder = getCANcoder();

    private final StatusSignal<Double> drivePosition = driveMotor.getPosition().clone();
    private final StatusSignal<Double> driveVelocity = driveMotor.getVelocity().clone();
    private final StatusSignal<Double> driveAppliedVolts = driveMotor.getMotorVoltage();
    private final StatusSignal<Double> driveCurrent = driveMotor.getStatorCurrent();

    private final StatusSignal<Double> steerPosition = steerMotor.getPosition().clone();
    private final StatusSignal<Double> steerVelocity = steerMotor.getVelocity().clone();
    private final StatusSignal<Double> steerAppliedVolts = steerMotor.getMotorVoltage();
    private final StatusSignal<Double> steerCurrent = steerMotor.getStatorCurrent();

    private final StatusSignal<Double> steerEncoderPosition = steerEncoder.getPosition().clone();
    private final StatusSignal<Double> steerEncoderVelocity = steerEncoder.getVelocity().clone();
    private final StatusSignal<MagnetHealthValue> steerEncoderAppliedVolts = steerEncoder.getMagnetHealth();

    public ThunderModule(SwerveModuleConstants constants, String canBusName, boolean supportsPro) {
        super(constants, canBusName, supportsPro);
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

        inputs.position = getPosition(true);
        inputs.state = lastState;

        inputs.runningOpenLoop = runningOpenLoop;
    }

    @Override
    public void apply(SwerveModuleState state, boolean isOpenLoop) {
        lastState = state;
        runningOpenLoop = isOpenLoop;
        super.apply(state, isOpenLoop);
    }
}