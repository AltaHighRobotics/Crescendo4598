// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Wacky little subsystem

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;

public class ShooterAndIntakeSub extends SubsystemBase {
  /** Creates a new ShooterSub. */
  private TalonFX shooterMotor;
  private TalonFX intakeMotor;

  public ShooterAndIntakeSub() {
    shooterMotor = new TalonFX(Constants.SHOOTER_MOTOR);
    intakeMotor = new TalonFX(Constants.INTAKE_MOTOR);

    shooterMotor.setNeutralMode(NeutralModeValue.Brake);
    intakeMotor.setNeutralMode(NeutralModeValue.Brake);
    shooterMotor.setInverted(true);
    intakeMotor.setInverted(true);

    //Remember to set one of the motors to be inverted.
  }

  public void setShooterMotor(double power) {
    shooterMotor.set(power);
  }

  public void setIntakeMotor(double power) {
    intakeMotor.set(power);
  }

  public double getShooterSpeed() {
    return shooterMotor.getVelocity().getValue();
  }

  public void stopShooter() {
    shooterMotor.stopMotor();
  }

  public void stopIntake() {
    intakeMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
