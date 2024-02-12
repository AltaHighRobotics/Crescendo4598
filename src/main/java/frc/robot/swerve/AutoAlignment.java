package frc.robot.swerve;

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

    public AutoAlignment(DriveTrainSub driveTrainSub) {
        m_driveTrainSub = driveTrainSub;

        positionPID = new ConfigurablePID(Constants.SWERVE_POSITION_PID);
        headingPID = new ConfigurablePID(Constants.SWERVE_HEADING_PID);
    }

    public void start(CartesianVector setpoint, double headingSetpoint) {
        this.setpoint = setpoint.clone();
        this.headingSetpoint = headingSetpoint;

        positionPID.resetValues();
        headingPID.resetValues();
    }

    public boolean run(CartesianVector processVarible, double heading, double robotYaw) {
        // Get direction and distance.
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

        boolean atPosition = Math.abs(positionPID.getError()) <= Constants.SWERVE_POSITION_THRESHOLD;

        // Heading stuff.
        double headingSpeed = 0.0;

        if (atPosition) {
            headingSpeed = -headingPID.runPID(headingSetpoint, heading);
            direction.x = 0.0;
            direction.y = 0.0;
        }

        // At position.
        if (atPosition && Math.abs(headingPID.getError()) <= Constants.SWERVE_HEADING_THRESHOLD) {
            m_driveTrainSub.drive(0.0, 0.0, 0.0, false, 0.0);
            return true;
        }

        // Run the drive.
        m_driveTrainSub.drive(direction.x, direction.y, headingSpeed, false, 1.0);
        m_driveTrainSub.run();

        return false;
    }
}
