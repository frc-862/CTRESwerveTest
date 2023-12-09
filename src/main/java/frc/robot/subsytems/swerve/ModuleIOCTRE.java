package frc.robot.subsytems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;

import frc.thunder.vision.Limelight;

public class ModuleIOCTRE extends SwerveModule implements ModuleIO {
    private final TalonFX driveMotor = getDriveMotor();
    private final TalonFX steerMotor = getSteerMotor();
    private final CANcoder steerEncoder = getCANcoder();

    private final StatusSignal<Double>[] signals = (StatusSignal<Double>[]) getSignals(); //Hopefully this will be made public in the future

    private final StatusSignal<Double> drivePosition = signals[0];
    private final StatusSignal<Double> driveVelocity = signals[1];
    private final StatusSignal<Double> driveAppliedVolts = driveMotor.getMotorVoltage();
    private final StatusSignal<Double> driveCurrent = driveMotor.getStatorCurrent();

    private final StatusSignal<Double> steerPosition = signals[2];
    private final StatusSignal<Double> steerVelocity = signals[3];
    private final StatusSignal<Double> steerAppliedVolts = steerMotor.getMotorVoltage();
    private final StatusSignal<Double> steerCurrent = steerMotor.getStatorCurrent();

    public ModuleIOCTRE(SwerveModuleConstants constants, String canBusName, boolean supportsPro) {
        super(constants, canBusName, supportsPro);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        BaseStatusSignal.refreshAll(signals);
        inputs.drivePositionDeg = drivePosition.getValueAsDouble() * 360d;
        inputs.driveVelocityRPM = driveVelocity.getValueAsDouble() / 60d;
        inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
        inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();
        inputs.steerPositionDeg = steerPosition.getValueAsDouble() * 360d;
        inputs.steerVelocityRPM = steerVelocity.getValueAsDouble() / 60d;
        inputs.steerAppliedVolts = steerAppliedVolts.getValueAsDouble();
        inputs.steerCurrentAmps = steerCurrent.getValueAsDouble();

        inputs.position = getPosition(true);
        inputs.state = getCurrentState();
    }

}
