package frc.robot.swerve;

import frc.robot.Constants;
import utilities.CartesianVector;
import utilities.ConfigurablePID;

// Relative alignment thingy mainly for limelight stuff.
public class AutoAlignment {
    private DriveTrainSub m_driveTrainSub;

    private CartesianVector setpoint;
    private ConfigurablePID positionPID;

    public AutoAlignment(DriveTrainSub driveTrainSub) {
        m_driveTrainSub = driveTrainSub;
        positionPID = new ConfigurablePID(Constants.SWERVE_POSITION_PID);
    }

    public void start(CartesianVector setpoint, double yawSetpoint) {
        this.setpoint = setpoint.clone();
        positionPID.resetValues();
    }

    public boolean run(CartesianVector processVarible) {
        // Get direction and distance.
        CartesianVector direction = setpoint.getSubtraction(processVarible);
        double distance = direction.magnitude2D();
        direction.normalize();

        // Get speed.
        double speed = positionPID.runPID(0.0, distance);
        direction.multiply(speed);

        // Rotate by yaw.
        double yaw = Math.toRadians(m_driveTrainSub.getYaw());
        double angleCos = Math.cos(yaw);
        double angleSin = Math.sin(yaw);

        double temp = direction.y * angleCos + direction.x * angleSin;
        direction.x = -direction.y * angleSin + direction.x * angleCos;
        direction.y = temp;

        // At position.
        if (Math.abs(positionPID.getError()) <= Constants.SWERVE_POSITION_THRESHOLD) {
            m_driveTrainSub.drive(0.0, 0.0, 0.0, false, 0.0);
            return true;
        }

        // Run the drive.
        m_driveTrainSub.drive(direction.x, direction.y, 0.0, false, 1.0);
        m_driveTrainSub.run();

        return false;
    }
}
