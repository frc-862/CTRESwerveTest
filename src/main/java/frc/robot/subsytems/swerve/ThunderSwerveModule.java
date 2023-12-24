package frc.robot.subsytems.swerve;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public class ThunderSwerveModule extends SwerveModule {

    public ThunderSwerveModule(SwerveModuleConstants constants, String canbusName) {
        super(constants, canbusName);
    }
    
    public void apply(int index, SwerveModuleState state, DriveRequestType driveRequestType, SteerRequestType steerRequestType) {
        super.apply(state, driveRequestType, steerRequestType);
        Logger.recordOutput("swerve/module/" + index + "/Steer Motor Control", getSteerMotor().getControlMode());
        Logger.recordOutput("swerve/module/" + index + "/Drive Motor Control", getDriveMotor().getControlMode());
    }
}
