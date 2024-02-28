// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import limelightvision.limelight.frc.LimeLight;
import limelightvision.limelight.frc.LimeLight.LimeLightTransform;
import utilities.CartesianVector;

public class VisionSub extends SubsystemBase {
  private LimeLight limeLight;

  // Used for finding the driveTrain position.
  private CartesianVector driveTrainPosition;
  private double driveTrainHeading = 0.0;

  public VisionSub() {
    limeLight = new LimeLight();
    setLimelightPipeline(Constants.LIMELIGHT_APRIL_TAG_PIPELINE);

    driveTrainPosition = new CartesianVector(0.0, 0.0);
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

  public LimeLightTransform getRobotPositionRed() {
    return limeLight.getRobotPositionRed();
  }

  public LimeLightTransform getAprilTagPositionRobotRelative() {
    return limeLight.getAprilTagPositionRobotRelative();
  }

  public double getAprilTagDistance() {
    LimeLightTransform transform = getAprilTagPositionRobotRelative();
    double distance = Math.sqrt(transform.x * transform.x 
    + transform.y * transform.y + transform.z * transform.z);

    return distance;
  }

  public void findDriveTrainPositionAndHeading() {
    LimeLightTransform transform = getRobotPosition();
    driveTrainPosition.x = transform.x + (16.54175 / 2.0);
    driveTrainPosition.y = transform.y + (8.21055 / 2.0);
    driveTrainHeading = 90.0 - transform.yaw;

    // 16.54175x8.21055 is field field sized
  }

  public CartesianVector getDriveTrainPosition() {
    return driveTrainPosition;
  }

  public double getDriveTrainHeading() {
    return driveTrainHeading;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Target found", getIsTargetFound());
    SmartDashboard.putNumber("Target distance", getAprilTagDistance());

    LimeLightTransform transform = getAprilTagPositionRobotRelative();
    SmartDashboard.putNumber("Limelight x", transform.x);
    SmartDashboard.putNumber("Limelight z", transform.z);
    SmartDashboard.putNumber("Limelight yaw", transform.pitch);
  }
}
