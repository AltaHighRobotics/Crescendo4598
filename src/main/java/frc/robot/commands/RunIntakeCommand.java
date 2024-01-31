// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterAndIntakeSub;

public class RunIntakeCommand extends Command {
  private ShooterAndIntakeSub m_shooterAndIntakeSub;

  public RunIntakeCommand(ShooterAndIntakeSub shooterAndIntakeSub) {
    m_shooterAndIntakeSub = shooterAndIntakeSub;

    addRequirements(m_shooterAndIntakeSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterAndIntakeSub.startShooterMoveCheck();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterAndIntakeSub.setIntakeMotor(Constants.INTAKE_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterAndIntakeSub.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean hasMoved = m_shooterAndIntakeSub.checkIfShooterHasMoved();
    return hasMoved;
  }
}
