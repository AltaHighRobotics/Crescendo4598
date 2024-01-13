// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.swerve.DriveTrainSub;
import utilities.CartesianVector;

public class TestAuto extends Command {
  private DriveTrainSub m_driveTrainSub;

  boolean done = false;
  int stage;

  public TestAuto(DriveTrainSub driveTrainSub) {
    m_driveTrainSub = driveTrainSub;

    addRequirements(m_driveTrainSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stage = 0;
    done = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean atPosition;

    switch (stage) {
      case 0:
        atPosition = m_driveTrainSub.driveTo(new CartesianVector(0.0, 1.2));

        if (atPosition) {
          stage = 1;
        }

        break;
      case 1:
        atPosition = m_driveTrainSub.driveTo(new CartesianVector(1.2, 1.2));

        if (atPosition) {
          done = true;
        }

        break;
      default:
        done = true;
        break;
    }

    SmartDashboard.putNumber("Stage", stage);
    SmartDashboard.putBoolean("Auto done", done);

    m_driveTrainSub.run();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
