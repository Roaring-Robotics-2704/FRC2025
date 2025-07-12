package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.auto.reef.Branch.Level;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running on a roboRIO. Change
 * the value of "simMode" to switch between "sim" (physics sim) and "replay" (log replay from a file).
 */
public final class Constants {
    public static final int DRIVE_CONTROLLER = 0;
    // Auto Priority
    public static final Level PRIORITY_LEVEL = Level.L4;

    // Controller
    public static final Boolean FieldRelative = true;

    // General Constants
    public static final double DRIVE_SPEED = 0.25;
    public static final double TURN_SPEED = 0.5;

    public static final Boolean COMPETITION = true;
    public static final Mode SIM_MODE = Mode.SIM;
    public static final Mode CURRENT_MODE = RobotBase.isReal() ? Mode.REAL : SIM_MODE;

    public enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static final int RECHECK_SECONDS = 12;


    /**
     * Returns the points scored for a given elevator level, depending on whether it's autonomous period.
     * @param level The elevator level
     * @param isAuto True if in autonomous (first 15 seconds), false otherwise
     * @return Points scored
     */
    public static int getPointsForLevel(Level level, boolean isAuto) {
        switch (level) {
            case L4:
                return isAuto ? 6 : 4;
            case L3:
                return isAuto ? 5 : 3;
            case L2:
                return isAuto ? 4 : 2;
            case L1:
                return isAuto ? 2 : 1;
            default:
                return 0;
        }
    }

    /**
     * Returns the points scored for a given elevator level (teleop, not autonomous).
     */
    public static int getPointsForLevel(Level level) {
        return getPointsForLevel(level, false);
    }

    /**
     * Returns the estimated time (in seconds) to raise the elevator to a given level.
     * These values should be tuned to your robot's actual performance.
     */
    public static double getElevatorTimeForLevel(Level level) {
        switch (level) {
            case L4:
                return 2.0;
            case L3:
                return 1.5;
            case L2:
                return 1.0;
            case L1:
                return 0.5;
            default:
                return 1.0;
        }
    }
}
