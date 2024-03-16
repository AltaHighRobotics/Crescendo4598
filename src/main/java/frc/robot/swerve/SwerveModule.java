package frc.robot.swerve;

import utilities.ConfigurablePID;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
  private static final double FORWARD = 1.0;
  private static final double BACKWARD = -1.0;

  // Wheel.
  private TalonFX wheelMotor;
  private double wheelDirection = FORWARD;

  private double lastWheelDistance = 0.0;
  private double distanceRate = 0.0;
  private double distance = 0.0;

  // Turn.
  private ConfigurablePID turnPid;

  private double desiredAngle = 0.0;
  private double turnAngle = 0.0;

  private CANSparkMax turnMotor;
  private RelativeEncoder turnEncoder;

  public SwerveModule(SwerveModuleConfig config) {
    // Wheel motor.
    wheelMotor = new TalonFX(config.wheelMotorId);

    // Config wheel motor.
    TalonFXConfiguration wheelMotorConfig = new TalonFXConfiguration();
    wheelMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    wheelMotorConfig.CurrentLimits.StatorCurrentLimit = Constants.SWERVE_MODULE_WHEEL_CURRENT_LIMIT;

    wheelMotor.getConfigurator().apply(wheelMotorConfig);

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
    wheelMotor.stopMotor();
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

  public ConfigurablePID getTurnPID() {
    return this.turnPid;
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

  public void setDesiredAngle(double desiredAngle) {
    this.turnAngle = desiredAngle;
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

  public double getTurnAngle() {
    return this.turnAngle;
  }

  public double getCurrentAngle() {
    double angle = getTurnEncoderPosition();
    
    if (wheelDirection == BACKWARD) {
      angle += 180.0;
    }

    angle = MathTools.wrapAngle(angle);
    return angle;
  }

  public double getWheelEncoder() {
    return wheelMotor.getPosition().getValue() * Constants.SWERVE_MODULE_WHEEL_ENCODER_DISTANCE_PER_PULSE;
  }

  public void resetWheelEncoder() {
    wheelMotor.setPosition(0.0);
  }

  // Tracks how far the wheel goes without a care of the direction.
  public double trackDistance() {
    double wheelDistance = getWheelEncoder();

    distanceRate = Math.abs(wheelDistance - lastWheelDistance);
    distance += distanceRate;
    lastWheelDistance = wheelDistance;

    return distance;
  }

  public double getDistanceRate() {
    return distanceRate;
  }

  public double getDistance() {
    return distance;
  }

  public void resetDistance() {
    lastWheelDistance = 0.0;
    distanceRate = 0.0;
    distance = 0.0;
    resetWheelEncoder();
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
