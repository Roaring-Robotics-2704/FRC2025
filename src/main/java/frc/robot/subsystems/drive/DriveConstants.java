package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

public class DriveConstants {
    public static final double MAX_SPEED = 4.8; // Meters per Second
    public static final double ODOMETRY_FREQUENCY = 100.0; // Hz
    public static final double TRACK_WIDTH = Units.inchesToMeters(26.5);
    public static final double TRACK_LENGTH = Units.inchesToMeters(26.5);
    public static final double DRIVE_BASE_RADIUS = Math.hypot(TRACK_WIDTH / 2.0, TRACK_LENGTH / 2.0);
    protected static final Translation2d[] moduleTranslations = new Translation2d[] {
        new Translation2d(TRACK_WIDTH / 2.0, TRACK_LENGTH / 2.0),
        new Translation2d(TRACK_WIDTH / 2.0, -TRACK_LENGTH / 2.0),
        new Translation2d(-TRACK_WIDTH / 2.0, TRACK_LENGTH / 2.0),
        new Translation2d(-TRACK_WIDTH / 2.0, -TRACK_LENGTH / 2.0)
    };

    // Zeroed rotation values for each module, see setup instructions
    public static final Rotation2d frontLeftZeroRotation = Rotation2d.fromDegrees(-90);
    public static final Rotation2d frontRightZeroRotation = Rotation2d.fromDegrees(0.0);
    public static final Rotation2d backLeftZeroRotation = Rotation2d.fromDegrees(180);
    public static final Rotation2d backRightZeroRotation = Rotation2d.fromDegrees(90);

    // Device CAN IDs
    public static final int PIGEON_CAN_ID = 9;

    public static final int FRONT_LEFT_DRIVE_CAN_ID = 1;
    public static final int BACK_LEFT_DRIVE_CAN_ID = 3;
    public static final int FRONT_RIGHT_DRIVE_CAN_ID = 2;
    public static final int BACK_RIGHT_DRIVE_CAN_ID = 4;

    public static final int FRONT_LEFT_TURN_CAN_ID = 5;
    public static final int BACK_LEFT_TURN_CAN_ID = 7;
    public static final int FRONT_RIGHT_TURN_CAN_ID = 6;
    public static final int BACK_RIGHT_TURN_CAN_ID = 8;

    // Drive motor configuration
    public static final int DRIVE_CURRENT_LIMIT = 45;
    public static final double WHEEL_RADIUS = Units.inchesToMeters(1.5); // meters
    public static final double DRIVE_REDUCTION = (45.0 * 22.0) / (14.0 * 15.0); // MAXSwerve with 14 pinion teeth
    // and 22 spur teeth
    public static final DCMotor DRIVE_GEARBOX = DCMotor.getNEO(1);

    // Drive encoder configuration
    public static final double DRIVE_POSITION_FACTOR = 2 * Math.PI / DRIVE_REDUCTION; // Rotor Rotations -> Wheel
    // Radians
    public static final double DRIVE_VELOCITY_FACTOR = (2 * Math.PI) / 60.0 / DRIVE_REDUCTION; // Rotor RPM -> Wheel
    // Rad/Sec

    // Drive PID configuration
    public static final double DRIVE_KP = 0.0;
    public static final double DRIVE_KD = 0.0;
    public static final double DRIVE_KS = 0.0;
    public static final double DRIVE_KV = 0.1;
    public static final double DRIVE_SIM_KP = 0.05;
    public static final double DRIVE_SIM_KD = 0.0;
    public static final double DRIVE_SIM_KS = 0.0;
    public static final double DRIVE_SIM_KV = 0.0789;

    // Turn motor configuration
    public static final boolean TURN_INVERTED = false;
    public static final int TURN_CURRENT_LIMIT = 20;
    public static final double TURN_REDUCTION = 9424.0 / 203.0;
    public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

    // Turn encoder configuration
    public static final boolean TURN_ENCODER_INVERTED = true;
    public static final double TURN_POSITION_FACTOR = 2 * Math.PI; // Rotations -> Radians
    public static final double TURN_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

    // Turn PID configuration
    public static final double TURN_KP = 2.0;
    public static final double TURN_KD = 0.0;
    public static final double TURN_SIM_KP = 8.0;
    public static final double TURN_SIM_KD = 0.0;
    public static final double TURN_MIN_INPUT = 0; // Radians
    public static final double TURN_MAX_INPUT = 2 * Math.PI; // Radians

    // PathPlanner configuration
    public static final double ROBOT_MASS = Units.lbsToKilograms(125); // KG
    public static final double ROBOT_MOI = 6.883;
    public static final double WHEEL_COF = 1.2;
    public static final RobotConfig ppConfig = new RobotConfig(
            ROBOT_MASS,
            ROBOT_MOI,
            new ModuleConfig(
                    WHEEL_RADIUS,
                    MAX_SPEED,
                    WHEEL_COF,
                    DRIVE_GEARBOX.withReduction(DRIVE_REDUCTION),
                    DRIVE_CURRENT_LIMIT,
                    1),
            moduleTranslations);

    public static final DriveTrainSimulationConfig mapleSimConfig = DriveTrainSimulationConfig.Default()
            .withCustomModuleTranslations(moduleTranslations)
            .withRobotMass(Kilogram.of(ROBOT_MASS))
            .withGyro(COTS.ofPigeon2())
            .withSwerveModule(new SwerveModuleSimulationConfig(
                    DRIVE_GEARBOX,
                    turnGearbox,
                    DRIVE_REDUCTION,
                    TURN_REDUCTION,
                    Volts.of(0.1),
                    Volts.of(0.1),
                    Meters.of(WHEEL_RADIUS),
                    KilogramSquareMeters.of(0.02),
                    WHEEL_COF));
    public static final PathConstraints CONSTRAINTS =
            new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
}
