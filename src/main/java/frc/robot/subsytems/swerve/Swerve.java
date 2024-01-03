package frc.robot.subsytems.swerve;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.TunerConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.RobotMap.CAN;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.vision.Limelight;
import frc.thunder.util.Pose4d;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class Swerve extends SwerveDrivetrain implements Subsystem {
    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();
    private Limelight[] limelights;

    public Swerve(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        this.limelights = new Limelight[] {
            new Limelight("limelight-back", "10.8.62.11"),
            new Limelight("limelight-front", "10.8.62.12")
        };


        configurePathPlanner();

        zeroGyro();

    }
    public Swerve(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        this(driveTrainConstants, 250, modules);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    @Override
    public void simulationPeriodic() {
        /* Assume  */
        updateSimState(0.02, 12);
    }

    @Override
    public void periodic() {
        for (Limelight limelight : Limelight.filterLimelights(limelights)) {
            Pose4d pose = limelight.getAlliancePose();
            addVisionMeasurement(pose.toPose2d(), Timer.getFPGATimestamp() - Units.millisecondsToSeconds(pose.getLatency()) - VisionConstants.PROCESS_LATENCY);
        }

        LightningShuffleboard.setDouble("Swerve", "yaw", m_yawGetter.getValueAsDouble());
        for (int i = 0; i < 4; ++i) {
            //TODO: refresh these signals???
            SwerveModule module = Modules[i];
            Logger.recordOutput("Swerve/Module" + i + "/DriveRotations", module.getDriveMotor().getPosition().getValueAsDouble());
            Logger.recordOutput("Swerve/Module" + i + "/DriveVelocityRPM", module.getDriveMotor().getVelocity().getValueAsDouble());
            Logger.recordOutput("Swerve/Module" + i + "/DriveMotorVolts", module.getDriveMotor().getMotorVoltage().getValueAsDouble());
            Logger.recordOutput("Swerve/Module" + i + "/DriveMotorCurrent", module.getDriveMotor().getStatorCurrent().getValueAsDouble());
            Logger.recordOutput("Swerve/Module" + i + "/DriveTempC", module.getDriveMotor().getDeviceTemp().getValueAsDouble());
            Logger.recordOutput("Swerve/Module" + i + "/SteerRotations", module.getSteerMotor().getPosition().getValueAsDouble());
            Logger.recordOutput("Swerve/Module" + i + "/SteerVelocityRPM", module.getSteerMotor().getVelocity().getValueAsDouble());
            Logger.recordOutput("Swerve/Module" + i + "/SteerMotorVolts", module.getSteerMotor().getMotorVoltage().getValueAsDouble());
            Logger.recordOutput("Swerve/Module" + i + "/SteerMotorCurrent", module.getSteerMotor().getStatorCurrent().getValueAsDouble());
            Logger.recordOutput("Swerve/Module" + i + "/SteerTempC", module.getSteerMotor().getDeviceTemp().getValueAsDouble());
            Logger.recordOutput("Swerve/Module" + i + "/EncoderRotations", module.getCANcoder().getPosition().getValueAsDouble());
            Logger.recordOutput("Swerve/Module" + i + "/EncoderRPM", module.getCANcoder().getVelocity().getValueAsDouble());
            Logger.recordOutput("Swerve/Module" + i + "/EncoderMagnet", module.getCANcoder().getMagnetHealth().getValue().toString());
        }
    }

    private void configurePathPlanner() {
        AutoBuilder.configureHolonomic(
            ()->this.getState().Pose, // Supplier of current robot pose
            this::seedFieldRelative,  // Consumer for seeding pose against auto
            this::getCurrentRobotChassisSpeeds,
            (speeds)->this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
            new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
                                            new PIDConstants(10, 0, 0),
                                            1,
                                            1,
                                            new ReplanningConfig(),
                                            0.004),
            this); // Subsystem for requirements
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public void zeroGyro() {
        m_pigeon2.setYaw(0);
    }

}