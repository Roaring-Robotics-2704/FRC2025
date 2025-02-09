package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.drive.DriveConstants.DRIVE_BASE_RADIUS;
import static frc.robot.subsystems.drive.DriveConstants.MAX_SPEED;
import static frc.robot.subsystems.drive.DriveConstants.moduleTranslations;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotContainer;
import frc.robot.subsystems.vision.Vision;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase implements Vision.VisionConsumer {
    public static final Lock odometryLock = new ReentrantLock();

    @SuppressWarnings("FieldMayBeFinal")
    private GyroIO gyroIO;

    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private final SwerveSetpointGenerator setpointGenerator;
    private SwerveSetpoint previousSetpoint;

    private final Module[] modules = new Module[4]; // FL, FR, BL, BR
    private final SysIdRoutine sysId;
    private final Alert gyroDisconnectedAlert =
            new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

    @SuppressWarnings("FieldMayBeFinal")
    private RobotConfig ppconfig;

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
    private Rotation2d rawGyroRotation = new Rotation2d();
    private final SwerveModulePosition[] lastModulePositions = // For delta tracking
            new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
            };
    private final SwerveDrivePoseEstimator poseEstimator =
            new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

    public Drive(GyroIO gyroIO, ModuleIO flModuleIO, ModuleIO frModuleIO, ModuleIO blModuleIO, ModuleIO brModuleIO) {
        local().gyroIO = gyroIO;
        modules[0] = new Module(flModuleIO, 0);
        modules[1] = new Module(frModuleIO, 1);
        modules[2] = new Module(blModuleIO, 2);
        modules[3] = new Module(brModuleIO, 3);

        local().ppconfig = DriveConstants.ppConfig;
        // try {
        // local().ppconfig = RobotConfig.fromGUISettings();
        // } catch (IOException | ParseException e) {
        // e.printStackTrace();
        // }

        // Usage reporting for swerve template
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

        // Start odometry thread
        SparkOdometryThread.getInstance().start();

        // Configure AutoBuilder for PathPlanner
        AutoBuilder.configure(
                local()::getPose,
                local()::resetOdometry,
                local()::getChassisSpeeds,
                local()::runVelocity,
                new PPHolonomicDriveController(new PIDConstants(7.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
                local().ppconfig,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                local());
        Pathfinding.setPathfinder(new LocalADStar());
        PathPlannerLogging.setLogActivePathCallback(
                activePath -> Logger.recordOutput("Odometry/Trajectory", activePath.toArray(Pose2d[]::new)));
        PathPlannerLogging.setLogTargetPoseCallback(
                targetPose -> Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose));

        // Configure SysId
        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null, state -> Logger.recordOutput("Drive/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(voltage -> runCharacterization(voltage.in(Volts)), null, local()));

        setpointGenerator = new SwerveSetpointGenerator(
                ppconfig, // The robot configuration. This is the same config used for generating
                // trajectories and running path following commands.
                Units.rotationsToRadians(10.0) // The max rotation velocity of a swerve module in radians per second.
                // This should probably be stored in your Constants file
                );

        // Initialize the previous setpoint to the robot's current speeds & module
        // states
        ChassisSpeeds currentSpeeds = getChassisSpeeds(); // Method to get current robot-relative chassis speeds
        SwerveModuleState[] currentStates = getModuleStates(); // Method to get the current swerve module states
        previousSetpoint =
                new SwerveSetpoint(currentSpeeds, currentStates, DriveFeedforwards.zeros(ppconfig.numModules));
    }

    private Drive local() {
        return this;
    }

    @Override
    public void periodic() {
        odometryLock.lock();
        try {
            // Prevents odometry updates while reading data
            gyroIO.updateInputs(gyroInputs);
            Logger.processInputs("Drive/Gyro", gyroInputs);
            for (var module : modules) {
                module.periodic();
            }
        } finally {
            odometryLock.unlock();
        }

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
        }

        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

        // Update odometry
        double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                        modulePositions[moduleIndex].distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
                        modulePositions[moduleIndex].angle);
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            // Update gyro angle
            if (gyroInputs.connected) {
                // Use the real gyro angle
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            // Apply update
            poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
        }

        // Update gyro alert
        gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.CURRENT_MODE != Mode.SIM);
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds) {
        // Calculate module setpoints
        speeds = ChassisSpeeds.discretize(speeds, 0.02);
        previousSetpoint = setpointGenerator.generateSetpoint(
                previousSetpoint, // The previous setpoint
                speeds, // The desired target speeds
                0.02 // The loop time of the robot code, in seconds
                );
        SwerveModuleState[] setpointStates = previousSetpoint.moduleStates();
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_SPEED);

        // Log unoptimized setpoints
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", speeds);

        // Send setpoints to modules
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i]);
        }

        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }

    public void runVelocityInverted(ChassisSpeeds speeds) {
        // Calculate module setpoints
        speeds = new ChassisSpeeds().minus(speeds);
        // Calculate module setpoints
        speeds = ChassisSpeeds.discretize(speeds, 0.02);
        previousSetpoint = setpointGenerator.generateSetpoint(
                previousSetpoint, // The previous setpoint
                speeds, // The desired target speeds
                0.02 // The loop time of the robot code, in seconds
                );
        SwerveModuleState[] setpointStates = previousSetpoint.moduleStates();
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_SPEED);

        // Log unoptimized setpoints
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", speeds);

        // Send setpoints to modules
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i]);
        }

        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }

    /** Runs the drive in a straight line with the specified drive output. */
    public void runCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(output);
        }
    }

    /** Stops the drive. */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will return to their
     * normal orientations the next time a nonzero velocity is requested.
     */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = moduleTranslations[i].getAngle();
        }
        kinematics.resetHeadings(headings);
        stop();
    }

    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.quasistatic(direction));
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
    }

    /** Returns the module states (turn angles and drive velocities) for all of the modules. */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /** Returns the module positions (turn angles and drive positions) for all of the modules. */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    /** Returns the measured chassis speeds of the robot. */
    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    /** Returns the position of each module in radians. */
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = modules[i].getWheelRadiusCharacterizationPosition();
        }
        return values;
    }

    /** Returns the average velocity of the modules in rad/sec. */
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += modules[i].getFFCharacterizationVelocity() / 4.0;
        }
        return output;
    }

    /** Returns the current odometry pose. */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    /** Resets the current odometry pose. */
    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
        if (Constants.CURRENT_MODE == Constants.Mode.SIM) {
            RobotContainer.resetSimulationField(pose);
        }
    }

    /** Adds a new timestamped vision measurement. */
    @Override
    public void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    /** Returns the maximum linear speed in meters per sec. */
    public double getMaxLinearSpeedMetersPerSec() {
        return MAX_SPEED;
    }

    /** Returns the maximum angular speed in radians per sec. */
    public double getMaxAngularSpeedRadPerSec() {
        return MAX_SPEED / DRIVE_BASE_RADIUS;
    }
}
