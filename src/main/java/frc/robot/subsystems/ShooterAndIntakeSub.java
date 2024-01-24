// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;

public class ShooterAndIntakeSub extends SubsystemBase {
  /** Creates a new ShooterSub. */
private TalonFX leftShooter;
private TalonFX rightShooter;

  public ShooterAndIntakeSub() {
    leftShooter = new TalonFX(Constants.LEFT_SHOOTER);
    rightShooter = new TalonFX(Constants.RIGHT_SHOOTER);

    //Remember to set one of the motors to be inverted.
  }

  public void runShooter() {
    leftShooter.set(Constants.SHOOT_SPEED);
    rightShooter.set(Constants.SHOOT_SPEED);
  }

  public void stopShooter() {
    leftShooter.stopMotor();
    rightShooter.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
