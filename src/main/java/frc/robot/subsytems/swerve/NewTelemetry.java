package frc.robot.subsytems.swerve;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class NewTelemetry {
    /* Keep a reference of the last pose to calculate the speeds */
    Pose2d m_lastPose = new Pose2d();
    double lastTime = Utils.getCurrentTimeSeconds();

    /* Accept the swerve drive state and telemeterize it to smartdashboard */
    public void telemeterize(SwerveDriveState state) {
        // Logger.recordOutput("Swerve/State", state);
        /* Telemeterize the pose */
        Pose2d pose = state.Pose;
        Logger.recordOutput("Swerve/Pose", pose);

        /* Telemeterize the robot's general speeds */
        double currentTime = Utils.getCurrentTimeSeconds();
        double diffTime = currentTime - lastTime;
        lastTime = currentTime;
        Translation2d distanceDiff = pose.minus(m_lastPose).getTranslation();
        m_lastPose = pose;

        Translation2d velocities = distanceDiff.div(diffTime);

        Logger.recordOutput("Swerve/Velocity", velocities.getNorm());
        Logger.recordOutput("Swerve/VelocityX", velocities.getX());
        Logger.recordOutput("Swerve/VelocityY", velocities.getY());
        Logger.recordOutput("Swerve/OdometryPeriod", state.OdometryPeriod);
    }
}
