package frc.robot.subsytems.swerve;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.VisionConstants;
import frc.thunder.util.Pose4d;
import frc.thunder.vision.Limelight;

public class NewSwerve extends SwerveDrivetrain implements Subsystem {
    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();
    private Limelight[] limelights;
    private final ModuleIOInputsAutoLogged[] inputs = new ModuleIOInputsAutoLogged[4];


    public NewSwerve(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        CommandScheduler.getInstance().registerSubsystem(this);
        this.limelights = new Limelight[] {
            // new Limelight("limelight-back", "10.8.62.11"),
            // new Limelight("limelight-front", "10.8.62.12")
        };

        configurePathPlanner();

        m_pigeon2.setYaw(0);

    }
    public NewSwerve(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
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
        m_pigeon2.setYaw(0); //TODO Make this work, get already created pigeon from drivetrain
    }

    @Override
    public void periodic() {
        for (Limelight limelight : Limelight.filterLimelights(limelights)) {
            Pose4d pose = limelight.getAlliancePose();
            addVisionMeasurement(pose.toPose2d(), Timer.getFPGATimestamp() - Units.millisecondsToSeconds(pose.getLatency()) - VisionConstants.PROCESS_LATENCY);
        }

        Logger.recordOutput("Swerve/yaw", m_yawGetter.getValueAsDouble());
        for (int i = 0; i < Modules.length; ++i) {

            (new ModuleIOCTRE(Modules[i])).updateInputs(inputs[i]);
            Logger.processInputs("Swerve/Module " + i, inputs[i]);
        }
    }
}
