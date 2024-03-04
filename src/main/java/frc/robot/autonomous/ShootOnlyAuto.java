// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterAndIntakeSub;

public class ShootOnlyAuto extends Command {
  private ShooterAndIntakeSub m_shooterAndIntakeSub;

  private boolean done;
  private long startTime;

  public ShootOnlyAuto(ShooterAndIntakeSub shooterAndIntakeSub) {
    m_shooterAndIntakeSub = shooterAndIntakeSub;

    addRequirements(m_shooterAndIntakeSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    done = false;
    startTime = -1;

    m_shooterAndIntakeSub.startShoot();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean isShootFinalStage = m_shooterAndIntakeSub.runShoot(Constants.SHOOTER_RYKEN_SPEED);

    // Start timer thingy at final stage.
    if (isShootFinalStage && startTime == -1) {
      startTime = System.currentTimeMillis();
    }

    // We is the done (:
    if (System.currentTimeMillis() - startTime >= 500 && startTime != -1) {
      m_shooterAndIntakeSub.endShoot();
      done = true;
    }

    SmartDashboard.putBoolean("Auto done", done);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterAndIntakeSub.endShoot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
