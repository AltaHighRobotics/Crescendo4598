// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.swerve.AutoAlignment;
import frc.robot.subsystems.ShooterAndIntakeSub;
import frc.robot.swerve.DriveTrainSub;
import utilities.CartesianVector;
import frc.robot.subsystems.VisionSub;

public class ChengSussyAuto extends Command {
  DriveTrainSub m_driveTrainSub;
  VisionSub m_visionSub;
  ShooterAndIntakeSub m_shooterAndIntakeSub;
  AutoAlignment m_autoAlignment;
  
  //stage for switch
  private int stage;
  private boolean done;

  /** Creates a new chengAutonomousCommand. */
  public ChengSussyAuto(DriveTrainSub driveTrainSub, VisionSub visionSub, ShooterAndIntakeSub shooterAndIntakeSub) {
    m_driveTrainSub = driveTrainSub;
    m_visionSub = visionSub;
    m_shooterAndIntakeSub = shooterAndIntakeSub;

    m_autoAlignment = new AutoAlignment(m_driveTrainSub);

    addRequirements(m_driveTrainSub, m_visionSub, m_shooterAndIntakeSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stage = 0;
    done = false;

    m_autoAlignment.start(new CartesianVector(0.0, 2.0), 0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(stage){
      case 0: 
        if(m_visionSub.getIsTargetFound() ){
          m_autoAlignment.run(new CartesianVector(stage, stage),11 );
        }
        stage = 1;
        break;
      case 1:
        if(m_visionSub.getIsTargetFound()){
          

          m_autoAlignment.run(new CartesianVector(stage, stage), m_visionSub.getAprilTagDistance());
        }
        if(m_autoAlignment.run(new CartesianVector(stage, stage), 11)){
          stage = 2;
        }
      case 2 :

      default:
        done = true;
        break;
    }

    m_driveTrainSub.run();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
