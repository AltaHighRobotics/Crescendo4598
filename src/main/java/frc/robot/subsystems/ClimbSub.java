// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

public class ClimbSub extends SubsystemBase {
  private TalonFX m_climbMotor;

  public ClimbSub() {
    m_climbMotor = new TalonFX(Constants.CLIMB_MOTOR);
    m_climbMotor.setInverted(true);
    m_climbMotor.setNeutralMode(NeutralModeValue.Brake);
    setClimbMotorPosition(0.0);
  }

  public void setClimbMotor(double power) {
    m_climbMotor.set(power);
  }

  public double getClimbMotorPosition() {
    return m_climbMotor.getPosition().getValue();
  }

  public void setClimbMotorPosition(double position) {
    m_climbMotor.setPosition(position);
  }

  public void stopClimbMotor() {
    m_climbMotor.stopMotor();
  }

  // Return true at limit.
  public boolean runClimbUp() {
    if (getClimbMotorPosition() < Constants.CLIMB_UPPER_LIMIT) {
      setClimbMotor(Constants.CLIMB_UP_SPEED);
      return false;
    } else {
      stopClimbMotor();
      return true;
    }
  }

  public boolean runClimbDown() {
    if (getClimbMotorPosition() > Constants.CLIMB_LOWER_LIMIT) {
      setClimbMotor(Constants.CLIMB_DOWN_SPEED);
      return false;
    } else {
      stopClimbMotor();
      return true;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climb position", getClimbMotorPosition());
  }
}
