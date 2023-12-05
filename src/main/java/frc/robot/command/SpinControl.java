// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsytems.Spin;

public class SpinControl extends Command {
  
  Spin spin;
  DoubleSupplier pow;

  /** Creates a new Spin. */
  public SpinControl(Spin spin, DoubleSupplier pow) {
    this.spin = spin;
    this.pow = pow;

    addRequirements(spin);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    spin.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    spin.setPower(pow.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    spin.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
