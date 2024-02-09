package frc.robot.swerve;

import utilities.CartesianVector;

// Relative alignment thingy mainly for limelight stuff.
public class AutoAlignment {
    private DriveTrainSub m_driveTrainSub;

    public AutoAlignment(DriveTrainSub driveTrainSub) {
        m_driveTrainSub = driveTrainSub;
    }

    public void start(CartesianVector setpoint, double yawSetpoint) {
        CartesianVector startPosition = m_driveTrainSub.getPosition();
        CartesianVector target = startPosition.getAddition(setpoint);
        double yaw = m_driveTrainSub.getYaw() + yawSetpoint;

        m_driveTrainSub.startDriveTo(target, yaw);
    }

    public boolean run() {
        boolean atPosition = m_driveTrainSub.driveTo();
        m_driveTrainSub.run();

        return atPosition;
    }
}
