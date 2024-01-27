// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.swerve.SwerveModuleConfig;
import utilities.PIDConfiguration;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // Driving.
  public static final int DRIVE_CONTROLLER = 0;
  public static final double DRIVE_SPEED = 0.5;
  public static final double DRIVE_TURN_SPEED = 0.5;

  //Shooter and Intake.
  public static final int SHOOTER_MOTOR = 4;
  public static final int INTAKE_MOTOR = 2;

  public static final double SHOOTER_HIGH_SPEED = 0.9;
  public static final double SHOOTER_LOW_SPEED = 0.22;
  public static final double INTAKE_SPEED = 0.37;

  // Controller.
  public static final double DRIVE_CONTROLLER_DEAD_ZONE = 0.15;

  public static final int RIGHT_STICK_Y = 3;
  public static final int RIGHT_STICK_X = 4;
  public static final int LEFT_STICK_Y = 1;
  public static final int LEFT_STICK_X = 0;

  public static final int FLIGHT_STICK_X = 0;
  public static final int FLIGHT_STICKY_Y = 1;
  public static final int FLIGHT_STICK_Z = 2;
  public static final int FLIGHT_STICK_SLIDER = 3;

  public static final int XBOX_A_BUTTON = 1; 
  public static final int XBOX_B_BUTTON = 2;
  public static final int XBOX_X_BUTTON = 3;
  public static final int XBOX_Y_BUTTON = 4;

  public static final int XBOX_LEFT_BUMPER = 5;
  public static final int XBOX_RIGHT_BUMPER = 6;

  // Swerve module.
  public static final double SWERVE_MODULE_TURN_ENCODER_DISTANCE_PER_PULSE = 2.099982500145832; // 42 steps per rotation old 12.600000000000001 171.43/1
  public static final double SWERVE_MODULE_WHEEL_CIRCUMFERENCE = 0.3092112569295754; // Meters
  public static final double SWERVE_MODULE_WHEEL_ENCODER_DISTANCE_PER_PULSE = 16384 * 0.00000826719 * 0.3092112569295754;
  // 16384 * gear_thingy * circumference. gear thingy is 0.00000826719.

  public static final double SWERVE_MODULE_WHEEL_CURRENT_LIMIT = 50.0;

  public static final PIDConfiguration SWERVE_MODULE_TURN_PID = new PIDConfiguration(0.012, 0.0, 0.0, 0.0, 0, 0, 0, 0, 0, 0, 0, -0.5, 0.5);

  public static final PIDConfiguration SWERVE_POSITION_PID = new PIDConfiguration(0.138, 0.0004, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.4, 0.4);
  public static final PIDConfiguration SWERVE_HEADING_PID = new PIDConfiguration(0.005, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.4, 0.4);
  public static final double SWERVE_POSITION_THRESHOLD = 0.05;
  public static final double SWERVE_HEADING_THRESHOLD = 5.0;

  public static final int FRONT_RIGHT_MODULE = 0;
  public static final int FRONT_LEFT_MODULE = 1;
  public static final int BACK_RIGHT_MODULE = 2;
  public static final int BACK_LEFT_MODULE = 3;
  public static final int SWERVE_MODULE_COUNT = 4;

  public static final SwerveModuleConfig []SWERVE_MODULE_CONFIGS = {
    new SwerveModuleConfig(20, 9, false, false), // Front right
    new SwerveModuleConfig(7, 3, true, false), // Front left
    new SwerveModuleConfig(32, 8, false, false), // Back right
    new SwerveModuleConfig(5, 6, true, false) // Back left
};
  
  public static final double VEHICLE_WHEELBASE = 1.0;
  public static final double VEHICLE_TRACKWIDTH = 1.0;
  public static final double VEHICLE_RADIUS = Math.hypot(VEHICLE_WHEELBASE, VEHICLE_TRACKWIDTH);
}
