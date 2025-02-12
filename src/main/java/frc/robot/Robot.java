// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ChutePos;
import frc.robot.Constants.ScoringPos;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
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
    private Command autonomousCommand;
    private final RobotContainer robotContainer;
    private final Field2d autofield = new Field2d();
    private final Field2d telefield = new Field2d();
    private String autoName;
    private Timer timer = new Timer();

    private static int scoringMode = 1;
    private static int pickupMode = 0;
    private static int chutePos = 0;

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

        // Start AdvantageKit logger
        Logger.start();
        if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
            Shuffleboard.selectTab("Autonomous");
        } else {
            Shuffleboard.selectTab("Simulation");
        }
        SmartDashboard.putData("Auto Field", autofield);
        SmartDashboard.putData("teleop Field", telefield);
        timer.start();

        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our autonomous chooser on the dashboard.
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
        // Return to normal thread priority
        Threads.setCurrentThreadPriority(false, 10);
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
        robotContainer.resetSimulationField();
    }

    /** This function is called periodically when disabled. */
    @Override
    @SuppressWarnings("CallToPrintStackTrace")
    public void disabledPeriodic() {
        if (timer.hasElapsed(Constants.RECHECK_SECONDS)) {
            updateAutoDisplay(autoName);
            timer.restart();
        }

        String newAutoName;
        newAutoName = robotContainer.getAutonomousCommand().getName();
        if (autoName == null ? (newAutoName != null) : !autoName.equals(newAutoName)) {
            autoName = newAutoName;
            updateAutoDisplay(autoName);
        }
    }

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
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

    public static ScoringPos getScoringMode() {
        return switch (scoringMode) {
            case 0 -> ScoringPos.Front;
            case 1 -> ScoringPos.FrontLeft;
            case 2 -> ScoringPos.FrontRight;
            case 3 -> ScoringPos.BackLeft;
            case 4 -> ScoringPos.BackRight;
            case 5 -> ScoringPos.Back;
            default -> ScoringPos.Back;
        };
    }

    public static void setScoringMode(ScoringPos mode) {
        switch (mode) {
            case Front -> scoringMode = 0;
            case FrontLeft -> scoringMode = 1;
            case FrontRight -> scoringMode = 2;
            case BackLeft -> scoringMode = 3;
            case BackRight -> scoringMode = 4;
            case Back -> scoringMode = 5;
            default -> scoringMode = 5;
        }
        Logger.recordOutput("ScoringMode", scoringMode);
    }

    public static ChutePos getChutePos() {
        return switch (chutePos) {
            case 0 -> ChutePos.FarLeft;
            case 1 -> ChutePos.MidLeft;
            case 2 -> ChutePos.NearLeft;
            case 3 -> ChutePos.NearRight;
            case 4 -> ChutePos.MidRight;
            case 5 -> ChutePos.FarRight;
            default -> ChutePos.FarRight;
        };
    }

    public static void setChutePos(ChutePos pos) {
        switch (pos) {
            case FarLeft -> chutePos = 0;
            case MidLeft -> chutePos = 1;
            case NearLeft -> chutePos = 2;
            case NearRight -> chutePos = 3;
            case MidRight -> chutePos = 4;
            case FarRight -> chutePos = 5;
            default -> chutePos = 0;
        }
        Logger.recordOutput("ChutePos", chutePos);
    }

    @SuppressWarnings("CallToPrintStackTrace")
    public void updateAutoDisplay(String name) {
        if (AutoBuilder.getAllAutoNames().contains(name)) {
            System.out.println("Displaying " + name);
            List<PathPlannerPath> pathPlannerPaths;
            try {
                pathPlannerPaths = PathPlannerAuto.getPathGroupFromAutoFile(name);

                List<Pose2d> poses = new ArrayList<>();
                for (PathPlannerPath path : pathPlannerPaths) {
                    poses.addAll(path.getAllPathPoints().stream()
                            .map(point -> new Pose2d(point.position.getX(), point.position.getY(), new Rotation2d()))
                            .collect(Collectors.toList()));
                }
                autofield.getObject("path").setPoses(poses);
                robotContainer.resetPose(poses.get(0));

            } catch (IOException | org.json.simple.parser.ParseException e) {
                DriverStation.reportError(e.getMessage(), false);
            }
        }
    }
}
