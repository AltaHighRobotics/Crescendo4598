package frc.robot.swerve;

public class SwerveModuleConfig {
    public int wheelMotorId;
    public int turnMotorId;
    public boolean invertWheelMotor;
    public boolean invertTurnMotor;

    public SwerveModuleConfig(int wheelMotorId, int turnMotorId, boolean invertWheelMotor, boolean invertTurnMotor) {
        this.wheelMotorId = wheelMotorId;
        this.turnMotorId = turnMotorId;
        this.invertWheelMotor = invertWheelMotor;
        this.invertTurnMotor = invertTurnMotor;
    }
}
