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
import frc.robot.swerve.AutoAlignment;
import frc.robot.Constants;

public class DriveShootFancypantsCommand extends Command {
  /** Creates a new DriveShootFancypantsCommand. */
  private DriveTrainSub m_driveTrainSub;
  private ShooterAndIntakeSub m_shooterAndIntakeSub;
  private VisionSub m_visionSub;
  private XboxController m_xboxController;

  private AutoAlignment autoAlignment;

  private int stage;
  private boolean done;

  private long startTime;

  public DriveShootFancypantsCommand(DriveTrainSub driveTrainSub, ShooterAndIntakeSub shooterAndIntakeSub, VisionSub visionSub, XboxController xboxController) {
    m_driveTrainSub = driveTrainSub;
    m_shooterAndIntakeSub = shooterAndIntakeSub;
    m_visionSub = visionSub;
    m_xboxController = xboxController;

    autoAlignment = new AutoAlignment(m_driveTrainSub);

    addRequirements(m_driveTrainSub, m_shooterAndIntakeSub, m_visionSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stage = 0;
    done = false;

    autoAlignment.start(new CartesianVector(0.0, 1.0), 0.0);
    m_driveTrainSub.setDriverControlEnabled(false);

    startTime = -1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // This no no worky.
    // if (m_xboxController.getRawButtonPressed(Constants.CODRIVER_AUTO_THINGY_BUTTON)) {
    //   done = true;
    // }

    boolean atPosition = false;

    switch (stage) {
      case 0: // Limelight align thingy
        // Run autoalignment if camera thingy seeee little pixel thingy
        if (m_visionSub.getIsTargetFound()) {
          LimeLightTransform transform = m_visionSub.getAprilTagPositionRobotRelative();
          atPosition = autoAlignment.run(new CartesianVector(transform.x, transform.z), transform.pitch);
        } else {
          m_driveTrainSub.drive(0.0, 0.0, 0.0, false, 0.0);
        }

        // Next stage
        if (atPosition) {
          m_driveTrainSub.resetPosition();
          m_driveTrainSub.setYaw(0.0);
          m_driveTrainSub.startDriveTo(new CartesianVector(0.0, 0.7), 0.0);
          stage = 1;
        }

        break;
      case 1: // drive align thingy
        atPosition = m_driveTrainSub.driveTo();

        if (atPosition) {
          stage = 2;
          m_shooterAndIntakeSub.startShoot();
        }

        break;
      case 2: // shoot
        boolean isShootFinalStage = m_shooterAndIntakeSub.runShoot(Constants.SHOOTER_TURTLE_SPEED);

        // Start timer thingy at final stage.
        if (isShootFinalStage && startTime == -1) {
          startTime = System.currentTimeMillis();
        }

        // We is the done (:
        if (System.currentTimeMillis() - startTime >= 500 && startTime != -1) {
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
    m_shooterAndIntakeSub.endShoot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
