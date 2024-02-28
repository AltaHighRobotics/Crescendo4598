// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.swerve.DriveTrainSub;
import frc.robot.subsystems.ShooterAndIntakeSub;
import frc.robot.subsystems.VisionSub;
import limelightvision.limelight.frc.LimeLight.LimeLightTransform;
import utilities.CartesianVector;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;

public class DriveShootFancypantsCommand extends Command {
  /** Creates a new DriveShootFancypantsCommand. */
  private DriveTrainSub m_driveTrainSub;
  private ShooterAndIntakeSub m_shooterAndIntakeSub;
  private VisionSub m_visionSub;
  private XboxController m_xboxController;

  int stage;
  boolean done;

  public DriveShootFancypantsCommand(DriveTrainSub driveTrainSub, ShooterAndIntakeSub shooterAndIntakeSub, VisionSub visionSub, XboxController xboxController) {
    m_driveTrainSub = driveTrainSub;
    m_shooterAndIntakeSub = shooterAndIntakeSub;
    m_visionSub = visionSub;
    m_xboxController = xboxController;

    addRequirements(m_driveTrainSub, m_shooterAndIntakeSub, m_visionSub);
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

    // This no no worky.
    // if (m_xboxController.getRawButtonPressed(Constants.CODRIVER_AUTO_THINGY_BUTTON)) {
    //   done = true;
    // }

    boolean atPosition;

    switch (stage) {
      case 0:

        // Get robo position or end thingy.
        if (m_visionSub.getIsTargetFound()) {
          // m_visionSub.findDriveTrainPositionAndHeading();
          // m_driveTrainSub.setPosition(m_visionSub.getDriveTrainPosition());
          // m_driveTrainSub.setYaw(m_visionSub.getDriveTrainHeading());
          stage = 1;

          LimeLightTransform transform = m_visionSub.getAprilTagPositionRobotRelative();
          m_driveTrainSub.setPosition(new CartesianVector(-transform.x, -transform.z));
          //m_driveTrainSub.setYaw(transform.pitch);
          m_driveTrainSub.setYaw(0.0);

          m_driveTrainSub.startDriveTo(new CartesianVector(0.0, 0.0), 0.0);
          m_driveTrainSub.setDriverControlEnabled(false);
        } else {
          done = true;
        }

        break;
      case 1:
        atPosition = m_driveTrainSub.driveTo();

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
  public void end(boolean interrupted) {
    m_driveTrainSub.drive(0.0, 0.0, 0.0, false, 0.0);
    m_driveTrainSub.setDriverControlEnabled(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
