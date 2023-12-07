// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.TunerConstants;
import frc.robot.subsytems.Spin;
import frc.robot.subsytems.Swerve;

public class RobotContainer {
  /* Setting up bindings for necessary control of the swerve drive platform */
  XboxController driver = new XboxController(0); // My joystick
  Swerve drivetrain = TunerConstants.DriveTrain; // My drivetrain
  Spin spin = new Spin();

  SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withIsOpenLoop(true); // I want field-centric driving in open loop
  SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  Telemetry logger = new Telemetry(Constants.MaxSpeed);

  // NamedCommands.registerCommand("spin20" ? () -> new SpinControl(spin, (() -> 20)));

  Command runAuto = drivetrain.getAutoPath("3in1");



  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-MathUtil.applyDeadband(driver.getLeftY(), 0.1) * Constants.MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-MathUtil.applyDeadband(driver.getLeftX(), 0.1) * Constants.MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-MathUtil.applyDeadband(driver.getRightX(), 0.1) * Constants.MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

	new Trigger(driver::getAButton).whileTrue(drivetrain.applyRequest(() -> brake));
	new Trigger(driver::getBButton).whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));
	new Trigger(driver::getXButton).onTrue(new InstantCommand(() -> drivetrain.zeroGyro())); // TODO create function to reset Heading
	new Trigger(driver::getRightBumper).onTrue(new InstantCommand(() -> drivetrain.slowMode())); // TODO create function to reset Heading
    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    // return Commands.print("No autonomous command configured");
    return runAuto;
  }
}
