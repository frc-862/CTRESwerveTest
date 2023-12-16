// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstatnts;
import frc.robot.subsytems.Swerve;

public class SlowMode extends Command {
  
	private Swerve drivetrain;

	DoubleSupplier LeftX;
	DoubleSupplier LeftY;
	DoubleSupplier RightX;

   SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();

  public SlowMode(DoubleSupplier LeftX, DoubleSupplier LeftY, DoubleSupplier RightX, Swerve drivetrain) {
	this.LeftX = LeftX;
	this.LeftY = LeftY;
	this.RightX = RightX;
	this.drivetrain = drivetrain;
	addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("MAde it");
    drivetrain.applyRequest(() -> drive.withVelocityX(-MathUtil.applyDeadband(LeftY.getAsDouble(), 0.1) * DrivetrainConstatnts.MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-MathUtil.applyDeadband(LeftX.getAsDouble(), 0.1) * DrivetrainConstatnts.MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-MathUtil.applyDeadband(RightX.getAsDouble(), 0.1) * DrivetrainConstatnts.MaxAngularRate *  DrivetrainConstatnts.RotationMultipler) // Drive counterclockwise with negative X (left)
        );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
