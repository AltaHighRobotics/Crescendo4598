// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.hal.simulation.ConstBufferCallback;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterAndIntakeSub;

public class ShootCommand extends Command
{
  private ShooterAndIntakeSub m_shooterAndIntakeSub;
  /** Creates a new ShootCommand. */
  private double speed;

  public ShootCommand(ShooterAndIntakeSub shooterAndIntakeSub, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooterAndIntakeSub = shooterAndIntakeSub;
    this.speed = speed;
    
    addRequirements(m_shooterAndIntakeSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterAndIntakeSub.setShooterMotor(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterAndIntakeSub.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}