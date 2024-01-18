package frc.robot.swerve;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.I2C.Port;
import utilities.CartesianVector;
import java.lang.Math;
import utilities.MathTools;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import utilities.ConfigurablePID;

public class DriveTrainSub extends SubsystemBase {
  /** Creates a new DriveTrainSub. */
  private SwerveModule[] swerveModuleSubs = new SwerveModule[Constants.SWERVE_MODULE_COUNT];
  private AHRS navx;

  private double fieldCentricOffset = 0.0;

  // Position stuff.
  private CartesianVector position;

  // Field.
  private Field2d field;

  // Controlling robot with points.
  private ConfigurablePID positionPID;
  private ConfigurablePID headingPID;

  public DriveTrainSub() {
    // Config swerve modules,
    for (int i = 0; i < Constants.SWERVE_MODULE_COUNT; ++i) {
      swerveModuleSubs[i] = new SwerveModule(Constants.SWERVE_MODULE_CONFIGS[i]);
    }

    // Gyro and field centric.
    navx = new AHRS(Port.kMXP);
    resetGyro();

    zeroFieldCentric();

    // Position and field.
    position = new CartesianVector(0.0, 0.0);
    field = new Field2d();

    positionPID = new ConfigurablePID(Constants.SWERVE_POSITION_PID);
    headingPID = new ConfigurablePID(Constants.SWERVE_HEADING_PID);
  }

  public void resetGyro() {
    navx.reset();
    navx.zeroYaw();
  }

  public double getFieldCentricYaw() {
    return getYaw() - fieldCentricOffset;
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

  private static double[] normalizeSpeeds(double[] speeds) {
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
    // Get rate and angle.
    double averageXRate = 0.0;
    double averageYRate = 0.0;

    double yaw = getYaw();

    for (SwerveModule module : swerveModuleSubs) {
      module.trackDistance();

      double turnAngle = module.getTurnAngle();
      double angle = Math.toRadians(yaw + turnAngle);
      double distanceRate = module.getDistanceRate();

      averageXRate += distanceRate * Math.sin(angle);
      averageYRate += distanceRate * Math.cos(angle);
    }

    // Add the rate.
    position.x += averageXRate / Constants.SWERVE_MODULE_COUNT;
    position.y += averageYRate / Constants.SWERVE_MODULE_COUNT;

    // Update field.
    field.setRobotPose(position.x, position.y, new Rotation2d(Math.toRadians(-yaw + 90.0)));
    SmartDashboard.putData("field", field);
  }

  public void setPosition(CartesianVector position) {
    this.position = position.clone();
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
    SmartDashboard.putNumber("Pitch", getPitch());
    SmartDashboard.putNumber("Roll", getRoll());
  }

  // Tell it to go to a position.
  public boolean driveTo(CartesianVector target, double targetHeading) {
    // Get direction and distance.
    CartesianVector direction = target.getSubtraction(position);
    double distance = direction.magnitude2D();
    direction.normalize();

    // At threhold.
    double heading = MathTools.makeNonNegAngle(getYaw());
    double headingError = Math.abs(targetHeading - heading);

    if (distance <= Constants.SWERVE_POSITION_THRESHOLD
      && headingError <= Constants.SWERVE_HEADING_THRESHOLD) {
      drive(0.0, 0.0, 0.0, false, 0.0);
      return true;
    }

    // Get speed.
    double speed = -positionPID.runPID(0.0, distance);
    direction.multiply(speed);

    // Rotate by yaw.
    double yaw = Math.toRadians(getYaw());
    double angleCos = Math.cos(yaw);
    double angleSin = Math.sin(yaw);

    double temp = direction.y * angleCos + direction.x * angleSin;
    direction.x = -direction.y * angleSin + direction.x * angleCos;
    direction.y = temp;

    // Heading.
    double headingSpeed = headingPID.runPID(targetHeading, heading);

    // Drive to point.
    drive(direction.x, direction.y, headingSpeed, false, 1.0);

    // Debug.
    SmartDashboard.putNumber("Drive to speed", speed);
    SmartDashboard.putNumber("Distance from target", distance);
    SmartDashboard.putNumber("Heading speed", headingSpeed);
    SmartDashboard.putNumber("Heading", heading);

    return false;
  }

  public void resetDriveTo() {
    positionPID.resetValues();
    headingPID.resetValues();
  }

  // Usefull stuff: https://www.chiefdelphi.com/uploads/default/original/3X/e/f/ef10db45f7d65f6d4da874cd26db294c7ad469bb.pdf
  public void drive(double strafe, double speed, double rotation, boolean fieldCentric, double driveSpeed) {
    double x = strafe;
    double y = speed;
    double z = rotation;

    // Field centric.
    if (fieldCentric) {
      double yaw = Math.toRadians(getFieldCentricYaw());
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