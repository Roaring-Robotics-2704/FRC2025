package frc.robot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.SystemTimeValidReader;
import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends LoggedRobot {

    private static final double loopOverrunWarningTimeout = 0.2;
    private static final double canErrorTimeThreshold = 0.5; // Seconds to disable alert
    private static final double canivoreErrorTimeThreshold = 0.5;
    private static final double lowBatteryVoltage = 11.8;
    private static final double lowBatteryDisabledTime = 1.5;
    private static final double lowBatteryMinCycleCount = 10;
    private static int lowBatteryCycleCount = 0;

    private Command autonomousCommand;
    private final RobotContainer robotContainer;
    private final Field2d autofield = new Field2d();
    private final Field2d telefield = new Field2d();
    LoggedPowerDistribution pdh;

    private double autoStart;
    private boolean autoMessagePrinted;
    private final Timer canInitialErrorTimer = new Timer();
    private final Timer canErrorTimer = new Timer();
    private final Timer disabledTimer = new Timer();

    private final Alert canErrorAlert =
            new Alert("CAN errors detected, robot may not be controllable.", AlertType.kError);
    private final Alert lowBatteryAlert = new Alert(
            "Battery voltage is very low, consider turning off the robot or replacing the battery.",
            AlertType.kWarning);
    private final Alert jitAlert = new Alert("Please wait to enable, JITing in progress.", AlertType.kWarning);

    public Robot() {
        // Record metadata
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0 -> Logger.recordMetadata("GitDirty", "All changes committed");
            case 1 -> Logger.recordMetadata("GitDirty", "Uncomitted changes");
            default -> Logger.recordMetadata("GitDirty", "Unknown");
        }

        // Set up data receivers & replay source
        switch (Constants.CURRENT_MODE) {
            case REAL -> {
                // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
            }

            case SIM -> // Running a physics simulator, log to NT
            Logger.addDataReceiver(new NT4Publisher());

            case REPLAY -> {
                // Replaying a log, set up replay source
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
            }
        }

        // Initialize URCL
        Logger.registerURCL(URCL.startExternal());
        pdh = LoggedPowerDistribution.getInstance(10, ModuleType.kRev);

        // Start AdvantageKit logger
        Logger.start();
        if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
            Shuffleboard.selectTab("Autonomous");
        } else {
            Shuffleboard.selectTab("Simulation");
        }
        SmartDashboard.putData("Auto Field", autofield);
        SmartDashboard.putData("teleop Field", telefield);
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our autonomous chooser on the dashboard.

        try {
            Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
            watchdogField.setAccessible(true);
            Watchdog watchdog = (Watchdog) watchdogField.get(this);
            watchdog.setTimeout(loopOverrunWarningTimeout);
        } catch (Exception e) {
            DriverStation.reportWarning("Failed to disable loop overrun warnings.", false);
        }
        CommandScheduler.getInstance().setPeriod(loopOverrunWarningTimeout);

        // Start system time valid reader
        SystemTimeValidReader.start();

        // Rely on our custom alerts for disconnected controllers
        DriverStation.silenceJoystickConnectionWarning(true);

        Map<String, Integer> commandCounts = new HashMap<>();
        BiConsumer<Command, Boolean> logCommandFunction = (Command command, Boolean active) -> {
            String name = command.getName();
            int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
            commandCounts.put(name, count);
            Logger.recordOutput("CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
            Logger.recordOutput("CommandsAll/" + name, count > 0);
        };
        CommandScheduler.getInstance()
                .onCommandInitialize((Command command) -> logCommandFunction.accept(command, true));
        CommandScheduler.getInstance().onCommandFinish((Command command) -> logCommandFunction.accept(command, false));
        CommandScheduler.getInstance()
                .onCommandInterrupt((Command command) -> logCommandFunction.accept(command, false));

        canInitialErrorTimer.restart();
        canErrorTimer.restart();
        disabledTimer.restart();

        // Configure brownout voltage
        RobotController.setBrownoutVoltage(6.0);

        robotContainer = new RobotContainer();
    }

    /** This function is called periodically during all modes. */
    @Override
    public void robotPeriodic() {
        // Switch thread to high priority to improve loop timing
        Threads.setCurrentThreadPriority(true, 99);

        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled commands, running already-scheduled commands, removing
        // finished or interrupted commands, and running subsystem periodic() methods.
        // This must be called from the robot's periodic block in order for anything in
        // the Command-based framework to work.
        CommandScheduler.getInstance().run();
        SmartDashboard.putNumber("MatchTime", DriverStation.getMatchTime());
        Logger.recordOutput("BatteryVoltage", RobotController.getBatteryVoltage());
        SmartDashboard.putBoolean("IsRedAlliance", isRedAlliance());
        SmartDashboard.putBoolean("Manual Mode", RobotContainer.isManual());

        // Check CAN status
        var canStatus = RobotController.getCANStatus();
        if (canStatus.transmitErrorCount > 0 || canStatus.receiveErrorCount > 0) {
            canErrorTimer.restart();
        }
        canErrorAlert.set(!canErrorTimer.hasElapsed(canErrorTimeThreshold)
                && !canInitialErrorTimer.hasElapsed(canErrorTimeThreshold));

        // Low battery alert
        lowBatteryCycleCount += 1;
        if (DriverStation.isEnabled()) {
            disabledTimer.reset();
        }
        if (RobotController.getBatteryVoltage() <= lowBatteryVoltage
                && disabledTimer.hasElapsed(lowBatteryDisabledTime)
                && lowBatteryCycleCount >= lowBatteryMinCycleCount) {
            lowBatteryAlert.set(true);
        }

        // JIT alert
        jitAlert.set(isJITing());
        // Return to normal thread priority
        Threads.setCurrentThreadPriority(false, 10);
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
        robotContainer.transition();
        // robotContainer.resetSimulationField();
    }

    /** This function is called periodically when disabled. */
    @Override
    @SuppressWarnings("CallToPrintStackTrace")
    public void disabledPeriodic() {

        // String newAutoName;
        // newAutoName = robotContainer.getAutonomousCommand().getName();
        // if (autoName == null ? (newAutoName != null) : !autoName.equals(newAutoName))
        // {
        // autoName = newAutoName;
        // // updateAutoDisplay(autoName);
        // }
    }

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
        if (Constants.CURRENT_MODE == Constants.Mode.SIM) {
            robotContainer.resetSimulationField();
        }
        autonomousCommand = robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        autofield.setRobotPose(robotContainer.getPose());
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
            Shuffleboard.selectTab("Teleoperated");
        }

        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        telefield.setRobotPose(robotContainer.getPose());
    }

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
        SimulatedArena.getInstance().simulationPeriodic();
        robotContainer.displaySimFieldToAdvantageScope();
    }

    public static boolean isRedAlliance() {
        return DriverStation.getAlliance()
                .filter(value -> value == DriverStation.Alliance.Red)
                .isPresent();
    }

    // @SuppressWarnings("CallToPrintStackTrace")
    // public void updateAutoDisplay(String name) {
    // if (AutoBuilder.getAllAutoNames().contains(name)) {
    // System.out.println("Displaying " + name);
    // List<PathPlannerPath> pathPlannerPaths;
    // try {
    // pathPlannerPaths = PathPlannerAuto.getPathGroupFromAutoFile(name);

    // List<Pose2d> poses = new ArrayList<>();
    // for (PathPlannerPath path : pathPlannerPaths) {
    // poses.addAll(path.getAllPathPoints().stream()
    // .map(point -> new Pose2d(point.position.getX(), point.position.getY(), new
    // Rotation2d()))
    // .collect(Collectors.toList()));
    // }
    // autofield.getObject("path").setPoses(poses);
    // robotContainer.resetPose(poses.get(0));

    // } catch (IOException | org.json.simple.parser.ParseException e) {
    // DriverStation.reportError(e.getMessage(), false);
    // }
    // }
    // }

    public static boolean isJITing() {
        return Timer.getTimestamp() < 45.0;
    }
}
