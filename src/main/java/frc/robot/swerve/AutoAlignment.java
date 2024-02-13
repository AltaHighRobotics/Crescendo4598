package frc.robot.swerve;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import utilities.CartesianVector;
import utilities.ConfigurablePID;

// Relative alignment thingy mainly for limelight stuff.
public class AutoAlignment {
    private DriveTrainSub m_driveTrainSub;

    private CartesianVector setpoint;
    private double headingSetpoint;

    private ConfigurablePID positionPID;
    private ConfigurablePID headingPID;

    private int stage;
    private boolean done;

    public AutoAlignment(DriveTrainSub driveTrainSub) {
        m_driveTrainSub = driveTrainSub;

        positionPID = new ConfigurablePID(Constants.SWERVE_POSITION_PID);
        headingPID = new ConfigurablePID(Constants.SWERVE_HEADING_PID);

        SmartDashboard.putData("Position thingy PID", positionPID);
        SmartDashboard.putData("heading thingy PID", headingPID);
    }

    public void start(CartesianVector setpoint, double headingSetpoint) {
        this.setpoint = setpoint.clone();
        this.headingSetpoint = headingSetpoint;

        positionPID.resetValues();
        headingPID.resetValues();

        stage = 0;
        done = false;
    }

    public boolean run(CartesianVector processVarible, double heading, double robotYaw) {
        if (done) {
            return true;
        }

        switch (stage) {
            case 0: // Align y.
                boolean atPositionY = alignY(processVarible, robotYaw);

                if (atPositionY) {
                    stage = 1;
                    positionPID.resetValues();
                }

                break;
            case 1: // Align x.
                boolean atPositionX = alignX(processVarible, robotYaw);

                if (atPositionX) {
                    stage = 2;
                }

                break;
            case 2: // Align heading.
                boolean atHeading = alignHeading(heading);

                if (atHeading) {
                    done = true;
                }
                
                break;
            default:
                done = true;
                break;
        }

        SmartDashboard.putNumber("Alignment stage", stage);

        return done;
    }

    private boolean alignPosition(CartesianVector processVarible, double robotYaw) {
        CartesianVector direction = setpoint.getSubtraction(processVarible);
        double distance = direction.magnitude2D();
        direction.normalize();

        // Get speed.
        double speed = positionPID.runPID(0.0, distance);
        direction.multiply(speed);

        // Rotate by yaw.
        double angleCos = Math.cos(robotYaw);
        double angleSin = Math.sin(robotYaw);

        double temp = direction.y * angleCos + direction.x * angleSin;
        direction.x = -direction.y * angleSin + direction.x * angleCos; 
        direction.y = temp;

        // At position.
        if (Math.abs(positionPID.getError()) <= 0.5) {
            m_driveTrainSub.drive(0.0, 0.0, 0.0, false, 0.0);
            return true;
        }

        // Run the drive.
        m_driveTrainSub.drive(direction.x, direction.y, 0.0, false, 1.0);

        return false;
    }

    private boolean alignX(CartesianVector processVarible, double robotYaw) {
        // Get speed.
        double speed = -positionPID.runPID(setpoint.x, processVarible.x);

        if (Math.abs(positionPID.getError()) <= 0.3) {
            m_driveTrainSub.drive(0.0, 0.0, 0.0, false, 0.0);
            return true;
        }

        CartesianVector direction = new CartesianVector(speed, 0.0);

         // Rotate by yaw.
        double angleCos = Math.cos(robotYaw);
        double angleSin = Math.sin(robotYaw);

        double temp = direction.y * angleCos + direction.x * angleSin;
        direction.x = -direction.y * angleSin + direction.x * angleCos; 
        direction.y = temp;

        // Run the drive.
        m_driveTrainSub.drive(speed, 0.0, 0.0, false, 1.0);

        return false;
    }

    private boolean alignY(CartesianVector processVarible, double robotYaw) {
        // Get speed.
        double speed = -positionPID.runPID(setpoint.y, processVarible.y);

        if (Math.abs(positionPID.getError()) <= 0.3) {
            m_driveTrainSub.drive(0.0, 0.0, 0.0, false, 0.0);
            return true;
        }

        CartesianVector direction = new CartesianVector(0.0, speed);

         // Rotate by yaw.
        double angleCos = Math.cos(robotYaw);
        double angleSin = Math.sin(robotYaw);

        double temp = direction.y * angleCos + direction.x * angleSin;
        direction.x = -direction.y * angleSin + direction.x * angleCos; 
        direction.y = temp;

        // Run the drive.
        m_driveTrainSub.drive(direction.x, direction.y, 0.0, false, 1.0);

        return false;
    }

    private boolean alignHeading(double heading) {
        double headingSpeed = -headingPID.runPID(headingSetpoint, heading);

        if (Math.abs(headingPID.getError()) <= Constants.SWERVE_HEADING_THRESHOLD) {
            m_driveTrainSub.drive(0.0, 0.0, 0.0, false, 0.0);
            return true;
        }

        m_driveTrainSub.drive(0.0, 0.0, headingSpeed, false, 1.0);

        return false;
    }
}
