// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.swerve.DriveTrainSub;

public class ResetDriveTrainStateCommand extends Command {
  private DriveTrainSub m_driveTrainSub;

  public ResetDriveTrainStateCommand(DriveTrainSub driveTrainSub) {
    m_driveTrainSub = driveTrainSub;

    addRequirements(m_driveTrainSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrainSub.setIsRunEnabled(false);
    m_driveTrainSub.drive(0.0, 0.0, 0.0, false, 0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrainSub.resetState();
    m_driveTrainSub.setIsRunEnabled(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
