// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.swerve.DriveTrainSub;
import frc.robot.subsystems.VisionSub;
import frc.robot.swerve.AutoAlignment;
import limelightvision.limelight.frc.LimeLight.LimeLightTransform;
import utilities.CartesianVector;;

public class TestLimelightAuto extends Command {
  private DriveTrainSub m_driveTrainSub;
  private VisionSub m_visionSub;
  private AutoAlignment autoAlignment;

  private boolean done;

  public TestLimelightAuto(DriveTrainSub driveTrainSub, VisionSub visionSub) {
    m_driveTrainSub = driveTrainSub;
    m_visionSub = visionSub;

    addRequirements(m_driveTrainSub, m_visionSub);
    // Use addRequirements() here to declare subsystem dependencies.

    autoAlignment = new AutoAlignment(m_driveTrainSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrainSub.resetGyro();
    m_driveTrainSub.resetPosition();

    autoAlignment.start(new CartesianVector(0.0, 2.0), 0.0);
    done = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    LimeLightTransform transform = m_visionSub.getAprilTagPositionRobotRelative();

    double yaw = Math.toRadians(m_driveTrainSub.getYaw());

    if (m_visionSub.getIsTargetFound()) {
      if (autoAlignment.run(new CartesianVector(transform.x, transform.z), transform.pitch)) {
        done = true;
      }
    } else {
      m_driveTrainSub.drive(0.0, 0.0, 0.0, false, 0.0);
    }

    m_driveTrainSub.run();

    SmartDashboard.putBoolean("Auto done", done);
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
