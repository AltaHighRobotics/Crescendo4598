// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Wacky little subsystem

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import frc.robot.Constants;
import utilities.ConfigurablePID;

public class ShooterAndIntakeSub extends SubsystemBase {
  /** Creates a new ShooterSub. */
  private TalonFX shooterMotor;
  private TalonFX intakeMotor;
  private WPI_VictorSPX roombaMotor;

  private double shooterPositionSinceCheck = 0.0;
  private boolean shooterHasMoved = false;

  private ConfigurablePID intakePID;
  private double intakeSetpoint = 0.0;

  // Shooty shoot.
  private int shootStage = 0;
  private long startTime = 0; // Can be used for whatever.

  public ShooterAndIntakeSub() {
    shooterMotor = new TalonFX(Constants.SHOOTER_MOTOR);
    intakeMotor = new TalonFX(Constants.INTAKE_MOTOR);
    roombaMotor = new WPI_VictorSPX(Constants.ROOMBA_MOTOR);

    // Motor settings.
    shooterMotor.setNeutralMode(NeutralModeValue.Brake);
    intakeMotor.setNeutralMode(NeutralModeValue.Brake);
    shooterMotor.setInverted(true);
    intakeMotor.setInverted(true);

    // Encoders.
    setIntakePosition(0.0);
    setShooterPosition(0.0);

    // Pid pid stuff lmao.
    intakePID = new ConfigurablePID(Constants.INTAKE_MOVE_PID);
    //SmartDashboard.putData("Intake pid", intakePID);
  }

  public void setShooterMotor(double power) {
    shooterMotor.set(power);
  }

  public void setIntakeMotor(double power) {
    intakeMotor.set(power);
    roombaMotor.set(VictorSPXControlMode.PercentOutput, power);
  }

  public void stopShooter() {
    shooterMotor.stopMotor();
  }

  public void stopIntake() {
    intakeMotor.stopMotor();
    roombaMotor.neutralOutput();
  }

  public double getIntakePosition() {
    return intakeMotor.getPosition().getValue() * Constants.INTAKE_ENCODER_DISTANCE_PER_PULSE;
  }

  public void setIntakePosition(double position) {
    intakeMotor.setPosition(position);
  }

  public double getShooterPosition() {
    return shooterMotor.getPosition().getValue() * Constants.SHOOTER_ENCODER_DISTANCE_PER_PULSE;
  }

  public double getShooterVelocity() {
    return shooterMotor.getVelocity().getValue() * Constants.SHOOTER_ENCODER_DISTANCE_PER_PULSE;
  }

  public void setShooterPosition(double position) {
    shooterMotor.setPosition(position);
  }

  // Stuff for checking if the shooter has moved since the last check.
  public void startShooterMoveCheck() {
    shooterPositionSinceCheck = getShooterPosition();
    shooterHasMoved = false;
  }

  public boolean checkIfShooterHasMoved() {
    if (Math.abs(getShooterPosition() - shooterPositionSinceCheck) >= Constants.SHOOTER_MOVE_THRESHOLD) {
      shooterHasMoved = true;
    }

    return shooterHasMoved;
  }

  // Stuff for moving the intake back.
  public void startIntakeMoveBack() {
    intakeSetpoint = getIntakePosition() - Constants.INTAKE_MOVE_BACK_BY;
    intakePID.resetValues();
  }

  public boolean moveIntakeBack() {
    // Run motor pid.
    double power = intakePID.runPID(intakeSetpoint, getIntakePosition());
    //SmartDashboard.putNumber("Intake setpoint", intakeSetpoint);

    // Is at position.
    if (Math.abs(intakePID.getError()) <= Constants.INTAKE_MOVE_THRESHOLD) {
      stopIntake();
      return true;
    }

    setIntakeMotor(power);

    return false;
  }

  // Fancy fancy shoot.
  public void startShoot(){
    shootStage = 0;

    stopShooter();
    stopIntake();

    startIntakeMoveBack();
  }

  public boolean runShoot(double speed) {
    boolean atLastStage = false;

    switch (shootStage) {
      case 0: // Move intake back.
        boolean movedBack = moveIntakeBack();

        if (movedBack) {
          shootStage = 1;
          startTime = System.currentTimeMillis();
        }

        break;
      case 1: // Speed up shooter.
        setShooterMotor(speed);

        if (System.currentTimeMillis() - startTime >= 200) {
          shootStage = 2;
        }

        break;
      case 2: // Load it in to shoot.
        setIntakeMotor(speed);
        atLastStage = true;
        break;
      default:
        atLastStage = true;
        break;
    }

    SmartDashboard.putNumber("Shoot stage", shootStage);

    return atLastStage;
  }

  public void endShoot() {
    stopShooter();
    stopIntake();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake position", getIntakePosition());
    SmartDashboard.putNumber("Shooter position", getShooterPosition());
    SmartDashboard.putNumber("Shooter velocity", getShooterVelocity());
  }
}
