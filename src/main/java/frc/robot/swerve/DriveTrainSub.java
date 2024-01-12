package frc.robot.swerve;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.I2C.Port;

import utilities.CartesianVector;

import java.lang.Math;
import utilities.MathTools;

public class DriveTrainSub extends SubsystemBase {
  /** Creates a new DriveTrainSub. */
  private SwerveModule[] swerveModuleSubs = new SwerveModule[Constants.SWERVE_MODULE_COUNT];
  private AHRS navx;

  private double fieldCentricOffset = 0.0;

  // Position stuff.
  private CartesianVector position;

  public DriveTrainSub() {
    // Config swerve modules,
    for (int i = 0; i < Constants.SWERVE_MODULE_COUNT; ++i) {
      swerveModuleSubs[i] = new SwerveModule(Constants.SWERVE_MODULE_CONFIGS[i]);
    }

    navx = new AHRS(Port.kMXP);
    resetGyro();

    zeroFieldCentric();

    position = new CartesianVector(0.0, 0.0);
  }

  public void resetGyro() {
    navx.reset();
    navx.zeroYaw();
  }

  public void zeroFieldCentric() {
    fieldCentricOffset = navx.getYaw();
  }

  public double getPitch() {
    return navx.getPitch();
  }

  public double getRoll() {
    return navx.getRoll();
  }

  public double getYaw() {
    return navx.getYaw();
  }

  public double getHeading() {
    return navx.getCompassHeading();
  }

  // Look in Constants.java for ids.
  public SwerveModule getSwerveModuleFromId(int id) {
    return swerveModuleSubs[id];
  }

  public SwerveModule[] getSwerveModules() {
    return swerveModuleSubs;
  }

  private static double[] normalizeSpeeds(double []speeds) {
    double[] normalizedSpeeds = speeds.clone();
    double max = normalizedSpeeds[0];

    // Get max.
    for (double v : normalizedSpeeds) {
      max = Math.max(max, v);
    }

    // Doesn't need to be normalized.
    if (max <= 1) {
      return normalizedSpeeds;
    }

    // Normalize.
    for (int i = 0; i < normalizedSpeeds.length; ++i) {
      normalizedSpeeds[i] /= max;
    }

    return normalizedSpeeds;
  }

  public void trackPosition() {
    // Get distance rate and angle.
    double averageDistanceRate = 0.0;
    double averageAngle = 0.0;

    for (SwerveModule module : swerveModuleSubs) {
      module.trackDistance();
      averageDistanceRate += module.getDistanceRate();
      averageAngle += module.getDesiredAngle();
    }

    averageDistanceRate /= Constants.SWERVE_MODULE_COUNT;
    averageAngle /= Constants.SWERVE_MODULE_COUNT;

    // Get x and y rate.
    double yaw = Math.toRadians(getYaw() + averageAngle);
    double yawCos = Math.cos(yaw);
    double yawSin = Math.sin(yaw);

    double x = 0.0;
    double y = averageDistanceRate;
    double xRate = -y * yawSin + x * yawCos;
    double yRate = y * yawCos + x * yawSin;

    //xRate *= averageDistanceRate;
    //yRate *= averageDistanceRate;

    // Add the rate.
    position.x += xRate;
    position.y += yRate;
  }

  public void resetPosition() {
    for (SwerveModule module : swerveModuleSubs) {
      module.resetDistance();
    }

    position.x = 0.0;
    position.y = 0.0;
  }

  public void run() {
    // Run the swerve module run methods.
    for (SwerveModule module : swerveModuleSubs) {
      module.run();
    }

    trackPosition();
    SmartDashboard.putNumber("x", position.x);
    SmartDashboard.putNumber("y", position.y);

    SmartDashboard.putNumber("Yaw", getYaw());
    SmartDashboard.putNumber("Front right distance", getSwerveModuleFromId(Constants.FRONT_RIGHT_MODULE).getDistance());
    SmartDashboard.putNumber("Front left distance", getSwerveModuleFromId(Constants.FRONT_LEFT_MODULE).getDistance());
    SmartDashboard.putNumber("Back right distance", getSwerveModuleFromId(Constants.BACK_RIGHT_MODULE).getDistance());
    SmartDashboard.putNumber("Back left distance", getSwerveModuleFromId(Constants.BACK_LEFT_MODULE).getDistance());
    SmartDashboard.putNumber("Pitch", getPitch());
    SmartDashboard.putNumber("Roll", getRoll());
  }

  // Usefull stuff: https://www.chiefdelphi.com/uploads/default/original/3X/e/f/ef10db45f7d65f6d4da874cd26db294c7ad469bb.pdf
  public void drive(double strafe, double speed, double rotation, boolean fieldCentric, double driveSpeed) {
    double x = strafe;
    double y = speed;
    double z = rotation;

    // Field centric.
    if (fieldCentric) {
      double yaw = Math.toRadians(getYaw() - fieldCentricOffset);
      double angleCos = Math.cos(yaw);
      double angleSin = Math.sin(yaw);

      double temp = y * angleCos + x * angleSin;
      x = -y * angleSin + x * angleCos;
      y = temp;
    }
  
    double a = x - z * (Constants.VEHICLE_WHEELBASE / Constants.VEHICLE_RADIUS);
    double b = x + z * (Constants.VEHICLE_WHEELBASE / Constants.VEHICLE_RADIUS);
    double c = y - z * (Constants.VEHICLE_TRACKWIDTH / Constants.VEHICLE_RADIUS);
    double d = y + z * (Constants.VEHICLE_TRACKWIDTH / Constants.VEHICLE_RADIUS);

    // Calculate module speeds.
    double[] moduleSpeeds = {
      Math.hypot(b, c),
      Math.hypot(b, d),
      Math.hypot(a, c),
      Math.hypot(a, d)
    };

    // Calculate module angles.
    double[] moduleAngles = {
      Math.atan2(b, c),
      Math.atan2(b, d),
      Math.atan2(a, c),
      Math.atan2(a, d)
    };

    // Normalize speeds.
    moduleSpeeds = normalizeSpeeds(moduleSpeeds);

    // Set speed and angle of each module.
    for (int i = 0; i < moduleSpeeds.length; ++i) {
      // Covert angle unit.
      moduleAngles[i] = MathTools.makeNonNegAngle(Math.toDegrees(moduleAngles[i]));

      swerveModuleSubs[i].setWheelMotor(moduleSpeeds[i] * driveSpeed);
      
      swerveModuleSubs[i].setDesiredAngle(moduleAngles[i]);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}