package utilities;

import edu.wpi.first.math.MathUtil;

public final class MathTools {

    // The gryo angle ranges from 180 to -180. This function convects that to a normal angle.
    public static final double makeNonNegAngle(double angle) {
        if (angle >= 0.0) {
            return angle;
        }

        return 360.0 + angle;
    }

    // Find the distance between two angles.
    public static final double angleDis(double a1, double a2) {
        double a, b;
        a = makeNonNegAngle(a1);
        b = makeNonNegAngle(a2);

        double dir = b - a;

        if (Math.abs(dir) > 180.0) {
            dir = -(Math.signum(dir) * 360.0) + dir;
        }

        return dir;
    }

    // Wrap angle to be from 0 to 360
    public static final double wrapAngle(double angle) {
        return MathUtil.inputModulus(angle, 0.0, 360.0);
    }

    // Find angle set point while handling angle wrapping.
    public static final double getAngleSetPoint(double desiredAngle, double currentAngle) {
        return currentAngle + angleDis(
            wrapAngle(currentAngle),
            desiredAngle
        );
    }
}