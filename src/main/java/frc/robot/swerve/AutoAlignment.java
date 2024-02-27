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

    // Don't forgor to run the drivetrain (:
    public boolean run(CartesianVector processVarible, double heading) {
        if (done) {
            return true;
        }

        boolean atPositionX = alignPosition(processVarible, heading);

        if (atPositionX) {
            done = true;
        }

        SmartDashboard.putNumber("Alignment stage", stage);

        return done;
    }

    private boolean alignPosition(CartesianVector processVarible, double heading) {
        CartesianVector direction = setpoint.getSubtraction(processVarible);
        double distance = direction.magnitude2D();
        direction.normalize();

        // Get speed.
        double speed = positionPID.runPID(0.0, distance);
        direction.multiply(speed);

        // Rotate by yaw.
        // double angleCos = Math.cos(-heading);
        // double angleSin = Math.sin(-heading);

        // double temp = direction.y * angleCos + direction.x * angleSin;
        // direction.x = -direction.y * angleSin + direction.x * angleCos; 
        // direction.y = temp;

        // Heading
        double headingSpeed = -headingPID.runPID(headingSetpoint, heading);

        // At position.
        if (Math.abs(positionPID.getError()) <= Constants.LIMLIGHT_POSITION_THRESHOLD &&
            Math.abs(headingPID.getError()) <= Constants.LIMELIGHT_HEADING_THRESHOLD) {
            m_driveTrainSub.drive(0.0, 0.0, 0.0, false, 0.0);
            return true;
        }

        // Run the drive.
        m_driveTrainSub.drive(direction.x, direction.y, headingSpeed, false, 1.0);

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
