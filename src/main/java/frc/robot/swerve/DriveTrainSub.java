package frc.robot.swerve;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.I2C.Port;
import utilities.CartesianVector;
import java.lang.Math;
import utilities.MathTools;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Scanner;
import java.io.FileNotFoundException;
import utilities.ConfigurablePID;

public class DriveTrainSub extends SubsystemBase {
  /** Creates a new DriveTrainSub. */
  private SwerveModule[] swerveModuleSubs = new SwerveModule[Constants.SWERVE_MODULE_COUNT];

  private AHRS navx;
  private double yawOffset = 0.0;
  private double fieldCentricOffset = 0.0;

  // Position stuff.
  private CartesianVector position;

  // Field.
  private Field2d field;

  // Controlling robot with points.
  private ConfigurablePID positionPID;
  private ConfigurablePID headingPID;

  // Target position.
  private CartesianVector targetPosition = new CartesianVector(0.0, 0.0);
  private double targetHeading = 0.0;

  // Roation stuff for auto driving. Tracks the rotations instead of angle. isn't limited to 0-360.
  private double lastHeading = 0.0;
  private double gyroRotation = 0.0;

  private boolean isRunEnabled = true;
  private boolean driverControlEnabled = true;

  public DriveTrainSub() {
    // Config swerve modules,
    for (int i = 0; i < Constants.SWERVE_MODULE_COUNT; ++i) {
      swerveModuleSubs[i] = new SwerveModule(Constants.SWERVE_MODULE_CONFIGS[i]);
    }

    // Link turn PIDS. The first one will be the master.
    // ConfigurablePID module1PID = swerveModuleSubs[0].getTurnPID();
    // module1PID.addLinkedPID(swerveModuleSubs[1].getTurnPID());
    // module1PID.addLinkedPID(swerveModuleSubs[2].getTurnPID());
    // module1PID.addLinkedPID(swerveModuleSubs[3].getTurnPID());

    //SmartDashboard.putData("Turn PID", module1PID);

    // Gyro and field centric.
    navx = new AHRS(Port.kMXP);
    resetGyro();

    zeroFieldCentric();

    // Position and field.
    position = new CartesianVector(0.0, 0.0);
    field = new Field2d();

    positionPID = new ConfigurablePID(Constants.SWERVE_POSITION_PID);
    headingPID = new ConfigurablePID(Constants.SWERVE_HEADING_PID);

    SmartDashboard.putData("Position PID", positionPID);
    SmartDashboard.putData("Heading PID", headingPID);
  }

  public void resetGyro() {
    navx.reset();
    navx.zeroYaw();

    fieldCentricOffset = 0.0;
    yawOffset = 0.0;
  }

  public double getFieldCentricYaw() {
    return getYaw() - fieldCentricOffset;
  }

  public void zeroFieldCentric() {
    fieldCentricOffset = navx.getYaw();
  }

  public void zeroFieldCentricReversed() {
    fieldCentricOffset = 180 - navx.getYaw();
  }

  public double getYaw() {
    return navx.getYaw() + yawOffset;
  }

  // *should* work (crying inside. i have no soul. i am not one, i am multiple. we are all multiple.)
  public void setYaw(double yaw) {
    //yawOffset = MathTools.angleDis(navx.getYaw(), yaw);
    //fieldCentricOffset += yawOffset;
  }

  // Look in Constants.java for ids.
  public SwerveModule getSwerveModuleFromId(int id) {
    return swerveModuleSubs[id];
  }

  public SwerveModule[] getSwerveModules() {
    return swerveModuleSubs;
  }

  public void resetState() {
    zeroFieldCentric();
    resetPosition();

    for (SwerveModule module : swerveModuleSubs) {
      module.resetTurnEncoder();
    }
  }

  // Saves info like encoder positions and other things I will add.
  public void saveState() {
    String filePath = "/home/lvuser/driveTrainState.data";

    try (PrintWriter textWriter = new PrintWriter(filePath)) {

      // Field centric.
      textWriter.println(fieldCentricOffset);

      // Position.
      textWriter.println(position.x);
      textWriter.println(position.y);

      // Encoder stuff.
      for (int i = 0; i < Constants.SWERVE_MODULE_COUNT; ++i) {
        textWriter.println(swerveModuleSubs[i].getTurnEncoderPosition());
      }
    } catch (IOException error) {
      System.out.println(error);
    }
  }

  // Loads that same info.
  public void loadState() {
    String filePath = "/home/lvuser/driveTrainState.data";
    File source = new File(filePath);

    try (Scanner textScanner = new Scanner(source)) {

      // Field centric.
      fieldCentricOffset = textScanner.nextDouble();
      textScanner.nextLine();

      // Position.
      position.x = textScanner.nextDouble();
      textScanner.nextLine();
      position.y = textScanner.nextDouble();
      textScanner.nextLine();

      // Debug.
      // System.out.println("Field centric offset: " + fieldCentricOffset);
      // System.out.println("Robot position x: " + position.x + ", y: " + position.y);

      // Encoder stuff.
      for (int i = 0; i < Constants.SWERVE_MODULE_COUNT; ++i) {
        // Ran out of lines toooo too soon ):
        if (!textScanner.hasNextLine()) {
          System.out.println("Turn position save has too few lines");
          break;
        }

        // Read value and eat up rest of the line.
        double encoderPosition = textScanner.nextDouble();
        //System.out.println("Model #" + i + " position " + encoderPosition);
        textScanner.nextLine();

        // Set it.
        swerveModuleSubs[i].setTurnEncoderPosition(encoderPosition);
      }
    } catch (FileNotFoundException error) {
      System.out.println(error);
    }
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

  private void trackPosition() {
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

  public CartesianVector getPosition() {
    return this.position.clone();
  }

  public void setPosition(CartesianVector position) {
    for (SwerveModule module : swerveModuleSubs) {
      module.resetDistance();
    }
    
    this.position = position.clone();
  }

  public void resetPosition() {
    for (SwerveModule module : swerveModuleSubs) {
      module.resetDistance();
    }

    position.set(0.0, 0.0);
  }

  public void setIsRunEnabled(boolean isRunEnabled) {
    this.isRunEnabled = isRunEnabled;
  }

  public boolean getIsRunEnabled() {
    return this.isRunEnabled;
  }

  public void run() {
    SmartDashboard.putBoolean("Swerve run enabled", isRunEnabled);

    // Disable the run thingy.
    if (!isRunEnabled) {
      return;
    }

    // Run the swerve module run methods.
    for (SwerveModule module : swerveModuleSubs) {
      module.run();
    }

    trackPosition();
    SmartDashboard.putNumber("x", position.x);
    SmartDashboard.putNumber("y", position.y);
    SmartDashboard.putNumber("Yaw", getYaw());
  }

  public boolean getDriverControlEnabled() {
    return this.driverControlEnabled;
  }

  public void setDriverControlEnabled(boolean driverControlEnabled) {
    this.driverControlEnabled = driverControlEnabled;
  }

  public void startDriveTo(CartesianVector targetPosition, double targetHeading) {
    this.targetPosition = targetPosition.clone();

    positionPID.resetValues();
    headingPID.resetValues();

    // Rotation setpoint because rotations are weird.
    double heading = MathTools.makeNonNegAngle(getYaw());
    lastHeading = heading;
    gyroRotation = heading;

    this.targetHeading = MathTools.getAngleSetPoint(targetHeading, heading);
  }

  // Tell it to go to a position.
  public boolean driveTo() {
    // Get direction and distance.
    CartesianVector direction = targetPosition.getSubtraction(position);
    double distance = direction.magnitude2D();
    direction.normalize();

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

    // Track rotations.
    double heading = MathTools.makeNonNegAngle(getYaw());
    gyroRotation += MathTools.angleDis(heading, lastHeading);
    lastHeading = heading;

    // Run heading pid.
    double headingSpeed = -headingPID.runPID(targetHeading, gyroRotation);

    // Is at position and heading.
    if (Math.abs(positionPID.getError()) <= Constants.SWERVE_POSITION_THRESHOLD
      && Math.abs(headingPID.getError()) <= Constants.SWERVE_HEADING_THRESHOLD) {
        drive(0.0, 0.0, 0.0, false, 0.0);
        return true;
    }

    // Drive to point.
    drive(direction.x, direction.y, headingSpeed, false, 1.0);

    // Debug.
    SmartDashboard.putNumber("Drive to speed", speed);
    SmartDashboard.putNumber("Distance from target", distance);
    SmartDashboard.putNumber("Heading", gyroRotation);
    SmartDashboard.putNumber("Heading setpoint", targetHeading);
    SmartDashboard.putNumber("Heading speed", headingSpeed);
    SmartDashboard.putNumber("x", position.x);
    SmartDashboard.putNumber("y", position.y);

    return false;
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