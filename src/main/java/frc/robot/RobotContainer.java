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

import static frc.robot.Constants.CONTROLLER;
import static frc.robot.subsystems.drive.DriveConstants.FINDINGCONSTRAINTS;
import static frc.robot.subsystems.drive.DriveConstants.PATHCONSTRAINTS;
import static frc.robot.subsystems.vision.VisionConstants.CAMERA_0_NAME;
import static frc.robot.subsystems.vision.VisionConstants.CAMERA_1_NAME;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera1;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.auto.reef.Branch.Level;
import frc.robot.auto.reef.Reef;
import frc.robot.auto.source.SourceChooser;
import frc.robot.auto.source.SourceChooser.SourceLocations;
import frc.robot.command_factories.ElevatorFactory;
import frc.robot.commands.autonomous.DynamicAuto;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.subsystems.buttonBoard.ButtonBoard;
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
import frc.robot.subsystems.elevator.ElevatorIOSpark;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.subsystems.outtake.OuttakeIO;
import frc.robot.subsystems.outtake.OuttakeIOSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.PoseUtil;
import java.util.function.Supplier;
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
    private Drive drive;

    @SuppressWarnings("unused")
    private Vision vision;

    private Elevator elevator;
    private Outtake outtake;

    private Reef reef = new Reef();
    private static SourceChooser sourceChooser = new SourceChooser();

    private static DynamicAuto dynamicAuto;
    // private static DynamicAutoBeta dynamicAutoBeta;

    private static SwerveDriveSimulation driveSimulation = null;

    // Controller
    private final CommandXboxController controller;
    private final CommandJoystick joystick;

    ButtonBoard buttonBoard = new ButtonBoard(reef);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Initialize Controller
        controller = new CommandXboxController(0);
        joystick = new CommandJoystick(2);

        switch (Constants.CURRENT_MODE) {
            case REAL: {
                // Real robot, instantiate hardware IO implementations
                drive = new Drive(
                        new GyroIOPigeon2(DriveConstants.PIGEON_CAN_ID),
                        new ModuleIOSpark(0),
                        new ModuleIOSpark(1),
                        new ModuleIOSpark(2),
                        new ModuleIOSpark(3));
                this.vision = new Vision(
                        drive,
                        new VisionIOPhotonVision(VisionConstants.CAMERA_0_NAME, VisionConstants.robotToCamera0),
                        new VisionIOPhotonVision(VisionConstants.CAMERA_1_NAME, VisionConstants.robotToCamera1));
                this.elevator = new Elevator(new ElevatorIOSpark());
                this.outtake = new Outtake(new OuttakeIOSpark());
                break;
            }

            case SIM: {
                // create a maple-sim swerve drive simulation instance
                driveSimulation =
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
                        new VisionIOPhotonVisionSim(
                                CAMERA_1_NAME, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose));

                this.elevator = new Elevator(new ElevatorIO() {});
                this.outtake = new Outtake(new OuttakeIO() {});
                break;
            }
            default: {
                // Replayed robot, disable IO implementations
                drive = new Drive(
                        new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
                vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});
                this.elevator = new Elevator(new ElevatorIO() {});
                this.outtake = new Outtake(new OuttakeIO() {});
                break;
            }
        }
        dynamicAuto = new DynamicAuto(reef, sourceChooser, drive);
        // dynamicAutoBeta = new DynamicAutoBeta(reef, sourceChooser, drive, elevator, outtake);

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
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
        autoChooser.addOption("Dynamic Auto", dynamicAuto);
        // autoChooser.addOption("Dynamic Auto Beta", dynamicAutoBeta);
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by instantiating a
     * {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
     * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        // Default command, normal field-relative drive
        if (CONTROLLER == Constants.Controller.XBOX) {
            drive.setDefaultCommand(DriveCommands.joystickDrive(
                    drive, () -> -controller.getLeftY(), () -> -controller.getLeftX(), () -> -controller.getRightX()));

        } else {
            drive.setDefaultCommand(DriveCommands.joystickDrive(
                    drive,
                    () -> -joystick.getRawAxis(1),
                    () -> -joystick.getRawAxis(0),
                    () -> -joystick.getRawAxis(2)));
        }

        // Switch to X pattern when X button is pressed
        if (CONTROLLER == Constants.Controller.XBOX) {
            controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
        } else {
            joystick.button(3).onTrue(Commands.runOnce(drive::stopWithX, drive));
            ;
        }
        // Reset gyro / odometry
        final Runnable resetGyro = Constants.CURRENT_MODE == Constants.Mode.SIM
                ? (() -> drive.resetOdometry(
                        driveSimulation
                                .getSimulatedDriveTrainPose())) // reset odometry to actual robot pose during simulation
                : (() -> drive.resetOdometry(
                        new Pose2d(drive.getPose().getTranslation(), new Rotation2d()))); // zero gyro

        if (CONTROLLER == Constants.Controller.XBOX) {
            controller.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));
        } else {
            joystick.button(4).onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));
        }
        // controller.a().whileTrue(new RunCommand(() ->
        // DriveCommands.goToSource(sourceChooser)));
        // controller
        // .y()
        // .whileTrue(new RunCommand(() -> DriveCommands.goToReef(reef,
        // buttonBoard.getSelectedBranchSide())));
        if (CONTROLLER == Constants.Controller.XBOX) {
            controller.a().whileTrue(Commands.deferredProxy(GoToReef()));
            controller.x().whileTrue(AutoBuilder.pathfindToPose(SourceLocations.SOURCE_LEFT, FINDINGCONSTRAINTS));
            controller.b().whileTrue(AutoBuilder.pathfindToPose(SourceLocations.SOURCE_RIGHT, FINDINGCONSTRAINTS));
            controller.y().whileTrue(dynamicAuto);
            controller.povDown().onTrue(ElevatorFactory.elevatorL1(elevator));
            controller.povLeft().onTrue(ElevatorFactory.elevatorL2(elevator));
            controller.povRight().onTrue(ElevatorFactory.elevatorL3(elevator));
            controller.povUp().onTrue(ElevatorFactory.elevatorL4(elevator));
        } else {
            joystick.button(1).whileTrue(Commands.deferredProxy(GoToReef()));
            joystick.button(2).whileTrue(Commands.deferredProxy(GoToSource()));
            joystick.button(5).whileTrue(AutoBuilder.pathfindToPose(SourceLocations.SOURCE_LEFT, FINDINGCONSTRAINTS));
            joystick.button(6).whileTrue(AutoBuilder.pathfindToPose(SourceLocations.SOURCE_RIGHT, FINDINGCONSTRAINTS));
            joystick.button(7).whileTrue(dynamicAuto);
        }
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public void resetSimulationField() {
        if (Constants.CURRENT_MODE != Constants.Mode.SIM) return;

        driveSimulation.setSimulationWorldPose(new Pose2d(8.125, 7.35, new Rotation2d()));
        SimulatedArena.getInstance().resetFieldForAuto();

        drive.resetOdometry(new Pose2d(8.125, 7.35, new Rotation2d()));
    }

    public static void resetSimulationField(Pose2d pose) {
        if (Constants.CURRENT_MODE != Constants.Mode.SIM) return;

        driveSimulation.setSimulationWorldPose(pose);
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

    public static Pose2d getBluePose() {
        if (Robot.isRedAlliance()) {
            return FlippingUtil.flipFieldPose(AutoBuilder.getCurrentPose());
        } else {
            return AutoBuilder.getCurrentPose();
        }
    }

    public Supplier<Command> GoToReef() {

        return () -> AutoBuilder.pathfindThenFollowPath(
                generatePath(
                                PoseUtil.offsetPose(
                                        reef.getclosestBranch(getPose(), Level.L3)
                                                .getPose(),
                                        -Units.feetToMeters(1),
                                        0),
                                reef.getclosestBranch(getPose(), Level.L3).getPose())
                        .get(),
                FINDINGCONSTRAINTS);
    }

    public static Supplier<PathPlannerPath> generatePath(Pose2d start, Pose2d end) {
        Rotation2d startRotation = new Rotation2d(Math.atan2(end.getY() - start.getY(), end.getX() - start.getX()));
        Rotation2d endRotation = end.getRotation();
        return () -> new PathPlannerPath(
                PathPlannerPath.waypointsFromPoses(
                        new Pose2d(start.getTranslation(), startRotation),
                        new Pose2d(end.getTranslation(), startRotation)),
                PATHCONSTRAINTS,
                new IdealStartingState(0.5, endRotation),
                new GoalEndState(0.0, endRotation)); // Goal end state with original rotation from end pose
    }

    public Supplier<Command> GoToSource() {

        return () -> AutoBuilder.pathfindThenFollowPath(
                generatePath(
                                PoseUtil.offsetPose(sourceChooser.getClosestSourcePose(), Units.feetToMeters(1), 0),
                                sourceChooser.getClosestSourcePose())
                        .get(),
                FINDINGCONSTRAINTS);
    }
}
