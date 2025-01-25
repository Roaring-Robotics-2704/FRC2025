package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running on a roboRIO. Change
 * the value of "simMode" to switch between "sim" (physics sim) and "replay" (log replay from a file).
 */
public final class Constants {
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

    public enum TargetHeightPos {
        Left,
        Center,
        Right
    }

    public enum ScoringPos {
        Back,
        BackLeft,
        BackRight,
        FrontLeft,
        FrontRight,
        Front
    }

    public enum ChutePos {
        FarLeft,
        MidLeft,
        NearLeft,
        FarRight,
        MidRight,
        NearRight
    }

}