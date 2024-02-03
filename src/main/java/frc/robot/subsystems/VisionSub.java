// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import limelightvision.limelight.frc.LimeLight;
import limelightvision.limelight.frc.LimeLight.LimeLightTransform;

public class VisionSub extends SubsystemBase {
  private LimeLight limeLight;

  public VisionSub() {
    limeLight = new LimeLight();
    setLimelightPipeline(Constants.LIMELIGHT_APRIL_TAG_PIPELINE);
  }

  public void setLimelightPipeline(int pipeline){
    limeLight.setPipeline(pipeline);
  }

  public int getID() {
    return limeLight.getAprilTagId();
  }

  public double getHorizontalOffset() {
    return limeLight.getdegRotationToTarget();
  }

  public double getVerticalOffset() {
    return limeLight.getTargetArea();
  }

  public boolean getIsTargetFound() {
    return limeLight.getIsTargetFound();
  }

  public LimeLightTransform getRobotPosition() {
    return limeLight.getRobotPosition();
  }

  public LimeLightTransform getRobotPositionBlue() {
    return limeLight.getRobotPositionBlue(); 
  }

  public LimeLightTransform getAprilTagPositionRobotRelative() {
    return limeLight.getAprilTagPositionRobotRelative();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Target found", getIsTargetFound());
    SmartDashboard.putData("Robot position", getRobotPosition());
  }
}
