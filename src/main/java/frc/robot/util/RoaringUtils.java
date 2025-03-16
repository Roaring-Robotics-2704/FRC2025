package frc.robot.util;

import edu.wpi.first.math.util.Units;

public class RoaringUtils {
    public static class DeadzoneUtils {
        public static double SquaredDeadband(double axis_value) {
            final double SCALE_EXPONENT = 2;
            final double DEADZONE_SIZE = 0.3;

            // creates scale factor based on deadzone size, so entire output area is still
            // usable
            double ScaleFactor = 1 / (1 - DEADZONE_SIZE);

            // x/|x| = 1 or -1 depending on sign of x
            double axis_sign = axis_value / Math.abs(axis_value);

            double unsigned_return_value = Math.abs(axis_value);

            unsigned_return_value = unsigned_return_value - DEADZONE_SIZE;

            if (unsigned_return_value < 0) { // set value to 0 if it's within the DEADZONE
                unsigned_return_value = 0;
            }
            unsigned_return_value = unsigned_return_value * ScaleFactor;

            unsigned_return_value = Math.pow(unsigned_return_value, SCALE_EXPONENT);
            if (Math.abs(axis_value) > DEADZONE_SIZE) {
                return axis_sign * unsigned_return_value;
            } else return 0;
        }

        public static double SquaredDeadband(double axis_value, double deadzone) {

            final double SCALE_EXPONENT = 2;
            final double DEADZONE_SIZE = Math.abs(deadzone);

            // creates scale factor based on deadzone size, so entire output area is still
            // usable
            double ScaleFactor = 1 / (1 - DEADZONE_SIZE);

            // x/|x| = 1 or -1 depending on sign of x
            double axis_sign = axis_value / Math.abs(axis_value);

            double unsigned_return_value = Math.abs(axis_value);

            unsigned_return_value = unsigned_return_value - DEADZONE_SIZE;

            if (unsigned_return_value < 0) { // set value to 0 if it's within the DEADZONE
                unsigned_return_value = 0;
            }
            unsigned_return_value = unsigned_return_value * ScaleFactor;

            unsigned_return_value = Math.pow(unsigned_return_value, SCALE_EXPONENT);
            if (Math.abs(axis_value) > DEADZONE_SIZE) {
                return axis_sign * unsigned_return_value;
            } else return 0;
        }

        public static double SquaredDeadband(double axis_value, double deadzone, double exp) {
            final double SCALE_EXPONENT = Math.abs(exp);
            final double DEADZONE_SIZE = Math.abs(deadzone);

            // creates scale factor based on deadzone size, so entire output area is still
            // usable
            double ScaleFactor = 1 / (1 - DEADZONE_SIZE);

            // x/|x| = 1 or -1 depending on sign of x
            double axis_sign = axis_value / Math.abs(axis_value);

            double unsigned_return_value = Math.abs(axis_value);

            unsigned_return_value = unsigned_return_value - DEADZONE_SIZE;

            if (unsigned_return_value < 0) { // set value to 0 if it's within the DEADZONE
                unsigned_return_value = 0;
            }
            unsigned_return_value = unsigned_return_value * ScaleFactor;

            unsigned_return_value = Math.pow(unsigned_return_value, SCALE_EXPONENT);
            if (Math.abs(axis_value) > DEADZONE_SIZE) {
                return axis_sign * unsigned_return_value;
            } else return 0;
        }

        public static double GenericDeadband(double raw, double deadzone) {

            if (Math.abs(raw) < deadzone) {
                return 0;
            } else {
                return raw;
            }
        }

        public static double LinearDeadband(double raw, double deadzone) {
            double c = deadzone;
            double slope = 1 / (1 - c);
            double sign = raw / Math.abs(raw);
            if (Math.abs(raw) > c) {
                return slope * (raw - (sign * c));
            } else {
                return 0;
            }

            // TODO add in linear deadband using slope claculation
        }
    }

    public static class GyroUtils {
        public static double WrapAngleDegreesHalf(double _angle) {
            double angle = Units.degreesToRadians(_angle);
            if (angle == 360) { // Handle this case separately to avoid floating point errors with the floor
                // after the division in the case below
                return 0.0;
            } else if (angle > 180) {
                return angle - 180 * Math.floor(angle / 180);
            } else if (angle < -180) {
                return angle + 180 * (Math.floor((-angle) / 180) + 1);
            } else {
                return angle;
            }
        }

        public static double WrapAngleRadiansFull(double _angle) {
            double twoPi = 2 * Math.PI;

            if (_angle == twoPi) { // Handle this case separately to avoid floating point errors with the floor
                // after the division in the case below
                return 0.0;
            } else if (_angle > twoPi) {
                return _angle - twoPi * Math.floor(_angle / twoPi);
            } else if (_angle < 0.0) {
                return _angle + twoPi * (Math.floor((-_angle) / twoPi) + 1);
            } else {
                return _angle;
            }
        }
    }

    public class POVDirections {
        public static final int POVTop = 0;
        public static final int POVTopRight = 45;
        public static final int POVRight = 90;
        public static final int POVBottomRight = 135;
        public static final int POVBottom = 180;
        public static final int POVBottomLeft = 225;
        public static final int POVLeft = 270;
        public static final int POVTopLeft = 315;
    }

    public class ToleranceChecker {
        public static boolean isWithinTolerance(double value1, double value2, double range) {
            return Math.abs(value1 - value2) <= range;
        }
    }
}
