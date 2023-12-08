// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsytems;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.thunder.config.FalconConfig;

public class Spin extends SubsystemBase {
  /** Creates a new Spin. */

  TalonFX motor;
  DoubleSupplier pow = () -> 0;

  public Spin() {
    this.motor = FalconConfig.createMotor(10, false, 0, 0, NeutralModeValue.Coast);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Spin/target power", pow.getAsDouble());

  }

  public void setPower(double pow){
    motor.set(pow);
    this.pow = () -> pow;
  }

  public void stop(){
    setPower(0d);
  }
}