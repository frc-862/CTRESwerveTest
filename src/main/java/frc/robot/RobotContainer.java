// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DrivetrainConstatnts;
import frc.robot.Constants.TunerConstants;
import frc.robot.subsytems.Spin;
import frc.robot.subsytems.collector.Collector;
import frc.robot.subsytems.collector.CollectorIOTalonFX;
import frc.robot.subsytems.swerve.NewSwerve;
import frc.robot.subsytems.swerve.Swerve;
import frc.robot.subsytems.swerve.Telemetry;
import frc.robot.subsytems.swerve.ThunderSwerveRequest;

public class RobotContainer {
  /* Setting up bindings for necessary control of the swerve drive platform */
  XboxController driver = new XboxController(ControllerConstants.DriverControllerPort); // My joystick
  NewSwerve drivetrain = TunerConstants.DriveTrain; // My drivetrain
}
  Spin spin = new Spin();
  Collector collector = new Collector(new CollectorIOTalonFX());


  ThunderSwerveRequest.FieldCentric drive = new ThunderSwerveRequest.FieldCentric(); //TODO I want field-centric driving in open loop   WE NEED TO FIGURE OUT WHAT Change beacuse with open loop is gone
  ThunderSwerveRequest.SwerveDriveBrake brake = new ThunderSwerveRequest.SwerveDriveBrake();
  ThunderSwerveRequest.PointWheelsAt point = new ThunderSwerveRequest.PointWheelsAt();
  // Telemetry logger = new Telemetry(DrivetrainConstatnts.MaxSpeed);

  // NamedCommands.registerCommand("spin20" ? () -> new SpinControl(spin, (() -> 20)));
//   NamedCommands.registerCommand("Flick", swerve.autoBalanceCommand());

  



  	private void configureBindings() {
  		// drivetrain.setDefaultCommand(new SlowMode(() -> driver.getLeftX(), () -> driver.getLeftY(), () -> driver.getRightY(), drivetrain));// TODO thisdoesnt curently work
	
		  drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
		  drivetrain.applyRequest(() -> drive.withVelocityX(-MathUtil.applyDeadband(driver.getLeftY(), 0.1) * DrivetrainConstatnts.MaxSpeed) // Drive forward with
																							 // negative Y (forward)
			  .withVelocityY(-MathUtil.applyDeadband(driver.getLeftX(), 0.1) * DrivetrainConstatnts.MaxSpeed) // Drive left with negative X (left)
			  .withRotationalRate(-MathUtil.applyDeadband(driver.getRightX(), 0.1) * DrivetrainConstatnts.MaxAngularRate) // Drive counterclockwise with negative X (left)
		  ));
		
		new Trigger(driver::getAButton).whileTrue(drivetrain.applyRequest(() -> brake));
		new Trigger(driver::getBButton).whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));
		new Trigger(driver::getXButton).onTrue(new InstantCommand(() -> drivetrain.zeroGyro())); // TODO create function to reset Heading
		// new Trigger(driver::getRightBumper).onTrue(new InstantCommand(() -> drivetrain.slowMode())); // TODO create function 
  
    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    // drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
    // return runAuto;
  }
}
