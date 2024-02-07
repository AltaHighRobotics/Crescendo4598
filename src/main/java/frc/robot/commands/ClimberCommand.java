// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSub;;

public class ClimberCommand extends Command {
  
  //make our sub
  private ClimbSub m_climbSub;
  
  //make our now time
  private final long start;
  /** Creates a new ClimberCommand. */
  public ClimberCommand(ClimbSub climbSub ) {
    // declare our subsystem
    m_climbSub = climbSub;

    //create time
    start = System.currentTimeMillis();

    addRequirements(m_climbSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
 //
  public boolean runToSecond(int second, long now){
   
    if((now - start)/1000 <= second ){
      return true;
    }
    else{
      return false;
    }
  }
  @Override
  public void execute() {
    long now = System.currentTimeMillis();
    if(runToSecond(3,now)){
      m_climbSub.grab();

    }
    else if (runToSecond(6, now)){
      m_climbSub.climb();
      
    }
    else{
      m_climbSub.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    m_climbSub.stop();
    return false;
  }
}
