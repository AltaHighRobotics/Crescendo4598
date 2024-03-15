// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.swerve.AutoAlignment;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterAndIntakeSub;
import frc.robot.swerve.DriveTrainSub;
import limelightvision.limelight.frc.LimeLight.LimeLightTransform;
import utilities.CartesianVector;
import frc.robot.subsystems.VisionSub;

public class TeamplayerAuto extends Command {
  DriveTrainSub m_driveTrainSub;
  VisionSub m_visionSub;
  ShooterAndIntakeSub m_shooterAndIntakeSub;
  AutoAlignment autoAlignment;
  
  private int stage;
  private boolean done;

  private long startTime;

  private boolean shooterMoveCheckStarted;

  public TeamplayerAuto(DriveTrainSub driveTrainSub, VisionSub visionSub, ShooterAndIntakeSub shooterAndIntakeSub) {
    m_driveTrainSub = driveTrainSub;
    m_visionSub = visionSub;
    m_shooterAndIntakeSub = shooterAndIntakeSub;

    autoAlignment = new AutoAlignment(m_driveTrainSub);

    addRequirements(m_driveTrainSub, m_visionSub, m_shooterAndIntakeSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stage = 0;
    done = false;

    startTime = -1;
    m_shooterAndIntakeSub.startShoot();

    shooterMoveCheckStarted = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean atPosition = false;
    boolean isShootFinalStage = false;
    boolean shooterHaveMoved = false;

    switch(stage) {
      case 0: // Shoot
        isShootFinalStage = m_shooterAndIntakeSub.runShoot(Constants.SHOOTER_RYKEN_SPEED);

        // Start timer thingy at final stage.
        if (isShootFinalStage && startTime == -1) {
          startTime = System.currentTimeMillis();
        }

        // We is the done (:
        if (System.currentTimeMillis() - startTime >= 500 && startTime != -1) {
          m_shooterAndIntakeSub.endShoot();

          // Drive back a bit.
          m_driveTrainSub.resetPosition();
          m_driveTrainSub.resetGyro();
          m_driveTrainSub.startDriveTo(new CartesianVector(0.0, -4.17), 60.0);

          stage = 1;
        }

        break;
      case 1: // Back out and try to pick up.
        atPosition = m_driveTrainSub.driveTo();
        m_shooterAndIntakeSub.setIntakeMotor(Constants.INTAKE_SPEED);
        shooterHaveMoved = false;
        
        // Wait for shooter to stop before checking if it has moved.
        if (shooterMoveCheckStarted) { // Check for shooter move and run intake.
          shooterHaveMoved = m_shooterAndIntakeSub.checkIfShooterHasMoved();
        } else if (m_shooterAndIntakeSub.getShooterVelocity() <= 0.0000001) {
          shooterMoveCheckStarted = true;
          m_shooterAndIntakeSub.startShooterMoveCheck();
        }
        
        // Next stage or end.
        if (shooterHaveMoved) {
          stage = 2;
          m_driveTrainSub.startDriveTo(new CartesianVector(0.0, -5.54), 0.0);
          m_shooterAndIntakeSub.stopIntake();
        } else if (atPosition) {
          done = true;
        }
      
        break;
      case 2: // Drive to shoot.
        atPosition = m_driveTrainSub.driveTo();

        if (atPosition) {
          m_shooterAndIntakeSub.startShoot();
          startTime = -1;
          stage = 3;
        }

        break;
      case 3: // shoot again.
        isShootFinalStage = m_shooterAndIntakeSub.runShoot(Constants.SHOOTER_RYKEN_SPEED);

        // Start timer thingy at final stage.
        if (isShootFinalStage && startTime == -1) {
          startTime = System.currentTimeMillis();
        }

        // More more we shall!
        if (System.currentTimeMillis() - startTime >= 500 && startTime != -1) {
          m_shooterAndIntakeSub.endShoot();
          m_driveTrainSub.startDriveTo(new CartesianVector(0.0, -0.0), 300.0);
          stage = 4;
        }

        break;
      case 4: // one more back out
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
    m_shooterAndIntakeSub.endShoot();
    m_shooterAndIntakeSub.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
