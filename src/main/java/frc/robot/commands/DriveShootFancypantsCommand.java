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
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.swerve.AutoAlignment;

import java.util.HashMap;
import java.util.Map;
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

  private boolean alignmentStarted;

  private double shootSpeed;

  // Lots o tags lmao
  private class TagAlignmentInfo {
    public CartesianVector position;
    public double heading;
    public double shootSpeed;

    public TagAlignmentInfo(CartesianVector position, double heading, double shootSpeed) {
      this.position = position;
      this.heading = heading;
      this.shootSpeed = shootSpeed;
    }
  }

  private Map<Integer, TagAlignmentInfo> tagIdInfo;

  public DriveShootFancypantsCommand(DriveTrainSub driveTrainSub, ShooterAndIntakeSub shooterAndIntakeSub, VisionSub visionSub, XboxController xboxController) {
    m_driveTrainSub = driveTrainSub;
    m_shooterAndIntakeSub = shooterAndIntakeSub;
    m_visionSub = visionSub;
    m_xboxController = xboxController;

    autoAlignment = new AutoAlignment(m_driveTrainSub);

    //SmartDashboard.putData("Position alignment PID", autoAlignment.getPositionPID());
    //SmartDashboard.putData("Heading alignment PID", autoAlignment.getHeadingPID());

    // Tag tag stuff
    tagIdInfo = new HashMap<>();
    tagIdInfo.put(5, new TagAlignmentInfo(new CartesianVector(0.0, 0.35), 0.0, 0.25));
    tagIdInfo.put(6, new TagAlignmentInfo(new CartesianVector(0.0, 0.35), 0.0, 0.25));

    for (int id = 11; id < 17; ++id)
    {
      tagIdInfo.put(id, new TagAlignmentInfo(new CartesianVector(-0.085, 0.928), -20.0, 0.8));
    }

    shootSpeed = Constants.SHOOTER_ELLA_SPEED;

    addRequirements(m_driveTrainSub, m_shooterAndIntakeSub, m_visionSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stage = 0;
    done = false;

    //autoAlignment.start(new CartesianVector(0.0, 0.35), 0.0);
    m_driveTrainSub.setDriverControlEnabled(false);
    m_shooterAndIntakeSub.startShoot();

    startTime = -1;
    alignmentStarted = false;
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

          if (alignmentStarted) { // Alignment stuff
            LimeLightTransform transform = m_visionSub.getAprilTagPositionRobotRelative();
            atPosition = autoAlignment.run(new CartesianVector(transform.x, transform.z), transform.pitch);
          } else { // Start alignment
            TagAlignmentInfo tagInfo = tagIdInfo.get(m_visionSub.getID());

            if (tagInfo != null) {
              alignmentStarted = true;
              autoAlignment.start(tagInfo.position, tagInfo.heading);
              shootSpeed = tagInfo.shootSpeed;
            }
          }
          
        } else {
          m_driveTrainSub.drive(0.0, 0.0, 0.0, false, 0.0);
        }

        // Next stage
        if (atPosition) {
          stage = 1;
        }

        break;
      case 1: // shoot
        boolean isShootFinalStage = m_shooterAndIntakeSub.runShoot(shootSpeed);

        // Start timer thingy at final stage.
        if (isShootFinalStage && startTime == -1) {
          startTime = System.currentTimeMillis();
        }

        // We is the done (:
        if (System.currentTimeMillis() - startTime >= 500 && startTime != -1) {
          m_shooterAndIntakeSub.endShoot();
          done = true;
        }
        
        break;
      default:
        done = true;
        break;
    }

    SmartDashboard.putNumber("Stage", stage);
    SmartDashboard.putBoolean("Auto done", done);
    // shootSpeed = SmartDashboard.getNumber("Shoot speed", Constants.SHOOTER_ELLA_SPEED);
    // SmartDashboard.putNumber("Shoot speed", shootSpeed);

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
