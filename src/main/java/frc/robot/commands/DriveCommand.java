// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.swerve.DriveTrainSub;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveCommand extends Command {
  private DriveTrainSub m_driveTrainSub;
  private XboxController m_driveController;

  private boolean doInitGyro = true;

  private boolean fieldCentric = true;

  public DriveCommand(DriveTrainSub driveTrainSub, XboxController driveController) {
    m_driveTrainSub = driveTrainSub;
    m_driveController = driveController;

    addRequirements(m_driveTrainSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // The gyro wants to be reset during runtime but I only want to do it once.
    if (doInitGyro) {
      doInitGyro = false;
      m_driveTrainSub.resetGyro();
      m_driveTrainSub.zeroFieldCentric();
    }

    // Get joystick values.
    double flightStickX = m_driveController.getRawAxis(Constants.FLIGHT_STICK_X);
    double flightStickY = m_driveController.getRawAxis(Constants.FLIGHT_STICKY_Y);
    double flightStickZ = m_driveController.getRawAxis(Constants.FLIGHT_STICK_Z);

    // Apply dead zones to controller.
    if (Math.abs(flightStickX) < Constants.DRIVE_CONTROLLER_DEAD_ZONE) {
      flightStickX = 0.0;
    } if (Math.abs(flightStickY) < Constants.DRIVE_CONTROLLER_DEAD_ZONE) {
      flightStickY = 0.0;
    } if (Math.abs(flightStickZ) < Constants.DRIVE_CONTROLLER_DEAD_ZONE) {
      flightStickZ = 0.0;
    }

    double strafe = flightStickX;
    double speed = flightStickY;
    double rotation = flightStickZ;

    // Toggle field centric.
    if (m_driveController.getRawButtonPressed(7)) {
      fieldCentric = !fieldCentric;
    }

    SmartDashboard.putBoolean("Field centric", fieldCentric);

    m_driveTrainSub.drive(
      Math.pow(strafe, 2.0) * Math.signum(strafe), 
      -Math.pow(speed, 2.0) * Math.signum(speed), 
      Math.pow(rotation, 2.0) * Math.signum(rotation) * Constants.DRIVE_TURN_SPEED,
      fieldCentric,
      Constants.DRIVE_SPEED
    );

    m_driveTrainSub.run();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
