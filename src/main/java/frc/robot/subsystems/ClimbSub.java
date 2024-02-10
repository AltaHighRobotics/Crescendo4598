// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSub extends SubsystemBase {
  private TalonFX climbMotor;
  
  /** Creates a new ClimbSub. */
  public ClimbSub() {
    climbMotor = new TalonFX(Constants.CLIMB_MOTOR);

    climbMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  //grab the chain and climb up 
  public  void climb() {
    climbMotor.set(Constants.CLIMB_SPEED);
  }

  //extend the claw to grab the chain
   public void grab() {
    climbMotor.set(-Constants.CLIMB_SPEED);
  }
 
  // stop after finishing motion
   public void stop(){
    climbMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
