package frc.robot.swerve;

import utilities.ConfigurablePID;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;

import frc.robot.Constants;
import utilities.MathTools;

public class SwerveModule {
  /** Creates a new SwerveModuleSub. */
  private SwerveModuleConfig configuration;

  private static final double FORWARD = 1.0;
  private static final double BACKWARD = -1.0;

  // Wheel.
  private TalonFX wheelMotor;
  private double wheelDirection = FORWARD;

  // Turn.
  private ConfigurablePID turnPid;
  private double desiredAngle = 0.0;

  private CANSparkMax turnMotor;
  private RelativeEncoder turnEncoder;

  public SwerveModule(SwerveModuleConfig config) {
    configuration = config;

    // Wheel motor.
    wheelMotor = new TalonFX(config.wheelMotorId);
    wheelMotor.setInverted(config.invertWheelMotor);
    wheelMotor.setNeutralMode(NeutralModeValue.Coast);
    resetWheelEncoder();

    // Turn motor.
    turnMotor = new CANSparkMax(config.turnMotorId, CANSparkLowLevel.MotorType.kBrushless);
    turnMotor.setIdleMode(IdleMode.kCoast);
    turnMotor.setInverted(config.invertTurnMotor);

    // // Turn encoder.
    turnEncoder = turnMotor.getEncoder();
    turnEncoder.setPositionConversionFactor(Constants.SWERVE_MODULE_TURN_ENCODER_DISTANCE_PER_PULSE);
    resetTurnEncoder();

    // Turn pid.
    turnPid = new ConfigurablePID(Constants.SWERVE_MODULE_TURN_PID);
  }

  public void setWheelMotor(double speed) {
    wheelMotor.set(speed * wheelDirection);
  }

  public void stopWheelMotor() {
    wheelMotor.stopMotor();;
  }

  public void setTurnMotor(double speed) {
    turnMotor.set(speed);
  }

  public void stopTurnMotor() {
    turnMotor.stopMotor();
  }

  public void stop() {
    stopWheelMotor();
    stopTurnMotor();
  }

  public void resetTurnEncoder() {
    turnEncoder.setPosition(0.0);
  }

  public double getAngle() {
    return MathTools.wrapAngle(getTurnEncoderPosition());
  }

  public double getTurnEncoderPosition() {
    return turnEncoder.getPosition();
  }

  public void setTurnEncoderPosition(double position) {
    turnEncoder.setPosition(position);
  }
  //I Love potato
  // tostinos tostinos hot pizza rolls
  //tostinos tostinos everybody's talking bout tostinos tostinos hot pizza rolls

  //ni how ma
  //hoa
  //robot worky pls
  //rock paper sciccor shoot

  // TODO: write me pleaz
  public void setDesiredAngle(double desiredAngle) {
    double turnDis = MathTools.angleDis(MathTools.wrapAngle(desiredAngle), getAngle());

    if (Math.abs(turnDis) > 90.0) {
      this.desiredAngle = MathTools.getAngleSetPoint(
        MathTools.wrapAngle(desiredAngle - turnDis), 
        getTurnEncoderPosition()
      );

      wheelDirection = BACKWARD;
    } else {
      this.desiredAngle = MathTools.getAngleSetPoint(desiredAngle, getTurnEncoderPosition());
      wheelDirection = FORWARD;
    }
  }

  public double getDesiredAngle() {
    return this.desiredAngle;
  }

  public double getDistance() {
    return wheelMotor.getPosition().getValue() * Constants.SWERVE_MODULE_WHEEL_ENCODER_DISTANCE_PER_PULSE;
  }

  public void resetWheelEncoder() {
    wheelMotor.setPosition(0.0);
  }

  public double getSpeed() {
    return wheelMotor.getVelocity().getValue() * Constants.SWERVE_MODULE_WHEEL_ENCODER_DISTANCE_PER_PULSE;
  }

  public double getAngleError() {
    return turnPid.getError();
  }
  
  public void run() {
    setTurnMotor(turnPid.runPID(desiredAngle, getTurnEncoderPosition()));
  }
}
