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
    public static final double DRIVE_SPEED = 0.5; // 0.5, 0.7
    public static final double TURN_SPEED = 0.5; // 0.5
    public static final double TEST_DRIVE_SPEED = 1;
    public static final double TEST_TURN_SPEED = 0.4;

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
}
