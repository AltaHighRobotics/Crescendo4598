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

  private double shooterPositionSinceCheck = 0.0;

  public ShooterAndIntakeSub() {
    shooterMotor = new TalonFX(Constants.SHOOTER_MOTOR);
    intakeMotor = new TalonFX(Constants.INTAKE_MOTOR);

    shooterMotor.setNeutralMode(NeutralModeValue.Brake);
    intakeMotor.setNeutralMode(NeutralModeValue.Brake);
    shooterMotor.setInverted(true);
    intakeMotor.setInverted(true);

    setShooterPosition(0.0);
  }

  public void setShooterMotor(double power) {
    shooterMotor.set(power);
  }

  public void setIntakeMotor(double power) {
    intakeMotor.set(power);
  }

  public void stopShooter() {
    shooterMotor.stopMotor();
  }

  public void stopIntake() {
    intakeMotor.stopMotor();
  }

  public double getShooterSpeed() {
    return shooterMotor.getVelocity().getValue();
  }

  public double getShooterPosition() {
    return shooterMotor.getPosition().getValue();
  }

  public void setShooterPosition(double position) {
    shooterMotor.setPosition(position);
  }

  public void startShooterMoveCheck() {
    shooterPositionSinceCheck = getShooterPosition();
  }

  public boolean hasShooterMoved() {
    return Math.abs(getShooterPosition() - shooterPositionSinceCheck) <= Constants.SHOOTER_MOVE_THRESHOLD;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter position", getShooterPosition());
  }
}
