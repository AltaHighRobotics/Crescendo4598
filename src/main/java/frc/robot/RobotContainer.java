// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants;
import frc.robot.autonomous.ChengSussyAuto;
import frc.robot.autonomous.TeamplayerAuto;
import frc.robot.autonomous.NothingAuto;
import frc.robot.autonomous.ShootOnlyAuto;
import frc.robot.autonomous.TestAuto;
import frc.robot.autonomous.TestLimelightAuto;

import java.io.CharArrayWriter;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.swerve.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Controllers.
  private final XboxController m_driveController = new XboxController(Constants.DRIVE_CONTROLLER);
  private final XboxController m_codriverController = new XboxController(Constants.CODRIVER_CONTROLLER);

  // Subsystems.
  private final DriveTrainSub m_driveTrainSub = new DriveTrainSub();
  private final ShooterAndIntakeSub m_shooterAndIntakeSub = new ShooterAndIntakeSub();
  private final VisionSub m_visionSub = new VisionSub();
  private final ClimbSub m_climbSub = new ClimbSub();

  // Commands.
  private final DriveCommand m_driveCommand = new DriveCommand(m_driveTrainSub, m_driveController);
  private final ResetFieldCentricCommand m_resetGyroCommand = new ResetFieldCentricCommand(m_driveTrainSub);

  private final ShootCommand m_shootRykenSpeedCommand = new ShootCommand(m_shooterAndIntakeSub, Constants.SHOOTER_RYKEN_SPEED);
  private final ShootCommand m_shootTurtleSpeedCommand = new ShootCommand(m_shooterAndIntakeSub, Constants.SHOOTER_TURTLE_SPEED);
  private final ShootCommand m_shootBuildteamSpeedCommand = new ShootCommand(m_shooterAndIntakeSub, Constants.SHOOTER_BUILDTEAM_SPEED);
  private final ShootCommand m_shootEllaSpeedCommand = new ShootCommand(m_shooterAndIntakeSub, Constants.SHOOTER_ELLA_SPEED);

  private final RunIntakeCommand m_runIntakeCommand = new RunIntakeCommand(m_shooterAndIntakeSub);

  private final SaveDriveTrainStateCommand m_saveDriveTrainStateCommand = new SaveDriveTrainStateCommand(m_driveTrainSub);
  private final LoadDriveTrainStateCommand m_loadDriveTrainStateCommand = new LoadDriveTrainStateCommand(m_driveTrainSub);
  private final ResetDriveTrainStateCommand m_resetDriveTrainStateCommand = new ResetDriveTrainStateCommand(m_driveTrainSub);

  private final ClimbUpCommand m_climbUpCommand = new ClimbUpCommand(m_climbSub);
  private final ClimbDownCommand m_climbDownCommand = new ClimbDownCommand(m_climbSub);

  private final RunIntakeBackwardsCommand m_runIntakeBackwardsCommand = new RunIntakeBackwardsCommand(m_shooterAndIntakeSub);

  private final DriveShootFancypantsCommand m_driveShootFancypantsCommand = new DriveShootFancypantsCommand(
    m_driveTrainSub, m_shooterAndIntakeSub, m_visionSub, m_codriverController);

  // Autonomous.
  private final TestAuto m_testAuto = new TestAuto(m_driveTrainSub);
  private final TestLimelightAuto m_testLimelightAuto = new TestLimelightAuto(m_driveTrainSub, m_visionSub);
  private final ChengSussyAuto m_chengSussyAuto = new ChengSussyAuto(m_driveTrainSub, m_visionSub, m_shooterAndIntakeSub);
  private final NothingAuto m_nothingAuto = new NothingAuto();
  private final ShootOnlyAuto m_shootOnlyAuto = new ShootOnlyAuto(m_shooterAndIntakeSub);
  private final TeamplayerAuto m_teamplayerAuto = new TeamplayerAuto(m_driveTrainSub, m_visionSub, m_shooterAndIntakeSub);

  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Add some buttons to the dash board.
    SmartDashboard.putData("Save drive train state", m_saveDriveTrainStateCommand);
    SmartDashboard.putData("Load drive train state", m_loadDriveTrainStateCommand);
    SmartDashboard.putData("Reset drive train state", m_resetDriveTrainStateCommand);

    // Auto chooser.
    m_autoChooser.setDefaultOption("Cheng sussy auto", m_chengSussyAuto);
    m_autoChooser.addOption("Team player", m_teamplayerAuto);
    m_autoChooser.addOption("Test limelight", m_testLimelightAuto);
    m_autoChooser.addOption("Test", m_testAuto);
    m_autoChooser.addOption("Sussy nothing", m_nothingAuto);
    m_autoChooser.addOption("Shoot only auto", m_shootOnlyAuto);
    SmartDashboard.putData("Auto chooser", m_autoChooser);

    // Configure the trigger bindings
    configureBindings();

    // Schedule drive command.
    CommandScheduler.getInstance().setDefaultCommand(m_driveTrainSub, m_driveCommand);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // The flight stick thingy thing has the (most) of the buttons labled with there id so using magic numbers is fine (:
    // If you don't agree then bring your opinion to your local walmart manager.
    // Define buttons.
    final JoystickButton resetGyroButton = new JoystickButton(m_driveController, 5);
    final JoystickButton runIntakeButton = new JoystickButton(m_driveController, 2);
    final JoystickButton autoThingyButton = new JoystickButton(m_driveController, 3);
    final JoystickButton runIntakeBackwardsButton = new JoystickButton(m_driveController, 4);

    final JoystickButton climbUpButton = new JoystickButton(m_codriverController, Constants.CLIMB_UP_BUTTON);
    final JoystickButton climbDownButton = new JoystickButton(m_codriverController, Constants.CLIMB_DOWN_BUTON);

    final JoystickButton rykenSpeedButton = new JoystickButton(m_codriverController, Constants.RYKEN_SPEED_BUTTON);
    final JoystickButton turtleSpeedButton = new JoystickButton(m_codriverController, Constants.TURTLE_SPEED_BUTTON);
    final JoystickButton buildteamSpeedButton = new JoystickButton(m_codriverController, Constants.BUILDTEAM_SPEED_BUTTON);
    final JoystickButton ellaSpeedButton = new JoystickButton(m_codriverController, Constants.ELLA_SPEED_BUTTON);

    final JoystickButton codriverRunIntakeButton = new JoystickButton(m_codriverController, Constants.CODRIVER_INTAKE_BUTTON);

    // Bind stuff.
    resetGyroButton.onTrue(m_resetGyroCommand);
    runIntakeButton.whileTrue(m_runIntakeCommand);
    autoThingyButton.whileTrue(m_driveShootFancypantsCommand);
    runIntakeBackwardsButton.whileTrue(m_runIntakeBackwardsCommand);
  
    // The based codriver.
    climbUpButton.whileTrue(m_climbUpCommand);
    climbDownButton.whileTrue(m_climbDownCommand);

    rykenSpeedButton.whileTrue(m_shootRykenSpeedCommand);
    turtleSpeedButton.whileTrue(m_shootTurtleSpeedCommand);
    buildteamSpeedButton.whileTrue(m_shootBuildteamSpeedCommand);
    ellaSpeedButton.whileTrue(m_shootEllaSpeedCommand);

    codriverRunIntakeButton.whileTrue(m_runIntakeCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_autoChooser.getSelected();
  }
}
