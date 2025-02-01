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

import static frc.robot.subsystems.vision.VisionConstants.CAMERA_0_NAME;
import static frc.robot.subsystems.vision.VisionConstants.CAMERA_1_NAME;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera1;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.auto.networktables.ReefChooser;
import frc.robot.auto.networktables.SourceChooser;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.autonomous.autos.DynamicAuto;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final Drive drive;
    private final Vision vision;
    private final Elevator elevator;
    private static SourceChooser sourceChooser = new SourceChooser();
    private static ReefChooser reefChooser = new ReefChooser();
    private DynamicAuto dynamicAuto;

    private SwerveDriveSimulation driveSimulation = null;

    // Controller
    private final CommandXboxController controller = new CommandXboxController(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        switch (Constants.CURRENT_MODE) {
            case REAL -> {
                // Real robot, instantiate hardware IO implementations
                this.elevator = new Elevator(new ElevatorIOSim());
                drive = new Drive(
                        new GyroIOPigeon2(DriveConstants.PIGEON_CAN_ID),
                        new ModuleIOSpark(0),
                        new ModuleIOSpark(1),
                        new ModuleIOSpark(2),
                        new ModuleIOSpark(3));
                this.vision = new Vision(
                        drive, new VisionIOLimelight(VisionConstants.CAMERA_0_NAME, drive::getRotation)
                        // new VisionIOLimelight(VisionConstants.CAMERA_1_NAME, drive::getRotation));
                        );
                dynamicAuto = new DynamicAuto(sourceChooser.getSourceChooser(), drive);
            }
            case SIM -> {
                elevator = new Elevator(new ElevatorIOSim());
                // create a maple-sim swerve drive simulation instance
                this.driveSimulation =
                        new SwerveDriveSimulation(DriveConstants.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
                // add the simulated drivetrain to the simulation field
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
                // Sim robot, instantiate physics sim IO implementations
                drive = new Drive(
                        new GyroIOSim(driveSimulation.getGyroSimulation()),
                        new ModuleIOSim(driveSimulation.getModules()[0]),
                        new ModuleIOSim(driveSimulation.getModules()[1]),
                        new ModuleIOSim(driveSimulation.getModules()[2]),
                        new ModuleIOSim(driveSimulation.getModules()[3]));
                vision = new Vision(
                        drive,
                        new VisionIOPhotonVisionSim(
                                CAMERA_0_NAME, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
                        new VisionIOPhotonVisionSim(CAMERA_1_NAME, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose));
                dynamicAuto = new DynamicAuto(sourceChooser.getSourceChooser(), drive);
            }
            default -> {
                elevator = new Elevator(new ElevatorIO() {});
                // Replayed robot, disable IO implementations
                drive = new Drive(
                        new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
                vision = new Vision(drive, new VisionIO() {});
                dynamicAuto = new DynamicAuto(sourceChooser.getSourceChooser(), drive);
            }
        }

        // Set up auto routines

        autoChooser = new LoggedDashboardChooser<>(
                "Auto Choices",
                AutoBuilder.buildAutoChooserWithOptionsModifier(stream -> Boolean.TRUE.equals(Constants.COMPETITION)
                        ? stream.filter(auto -> auto.getName().startsWith("comp"))
                        : stream));

        if (Boolean.FALSE.equals(Constants.COMPETITION)) {
            // Set up SysId routines
            autoChooser.addOption(
                    "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
            autoChooser.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
            autoChooser.addOption(
                    "Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
            autoChooser.addOption(
                    "Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
            autoChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
            autoChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        }
        // dynamicAuto.schedule();
        // Configure the button bindings
        configureButtonBindings();
        FollowPathCommand.warmupCommand();
        PathfindingCommand.warmupCommand();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by instantiating a
     * {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
     * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(DriveCommands.joystickDrive(
                drive, () -> -controller.getLeftY(), () -> -controller.getLeftX(), () -> -controller.getRightX()));

        // Lock to 0° when A button is held
        controller
                .a()
                .whileTrue(DriveCommands.joystickDriveAtAngle(
                        drive, () -> -controller.getLeftY(), () -> -controller.getLeftX(), Rotation2d::new));

        // Switch to X pattern when X button is pressed
        controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

        // Reset gyro / odometry
        final Runnable resetGyro = Constants.CURRENT_MODE == Constants.Mode.SIM
                ? (() -> drive.resetOdometry(driveSimulation.getSimulatedDriveTrainPose())) // reset odometry to
                // actual robot pose
                // during simulation
                : (() -> drive.resetOdometry(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()))); // zero
        // gyro
        controller.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return dynamicAuto;
    }

    public void resetSimulationField() {
        if (Constants.CURRENT_MODE != Constants.Mode.SIM) return;

        driveSimulation.setSimulationWorldPose(new Pose2d(8.125, 7.35, new Rotation2d()));
        SimulatedArena.getInstance().resetFieldForAuto();
        drive.resetOdometry(new Pose2d(8.125, 7.35, new Rotation2d()));
    }

    public void displaySimFieldToAdvantageScope() {
        if (Constants.CURRENT_MODE != Constants.Mode.SIM) return;

        Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput(
                "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
        Logger.recordOutput(
                "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    }

    public Pose2d getPose() {
        return drive.getPose();
    }

    public void resetPose(Pose2d pose) {
        drive.resetOdometry(pose);
    }
}
