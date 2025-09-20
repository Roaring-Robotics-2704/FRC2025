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

import static frc.robot.Constants.*;
import static frc.robot.subsystems.buttonBoard.ButtonBoardConstants.BB_PORT;
import static frc.robot.subsystems.drive.DriveConstants.FINDINGCONSTRAINTS;
import static frc.robot.subsystems.drive.DriveConstants.PATHCONSTRAINTS;
import static frc.robot.subsystems.vision.VisionConstants.CAMERA_0_NAME;
import static frc.robot.subsystems.vision.VisionConstants.CAMERA_1_NAME;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera1;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.auto.reef.Branch.Level;
import frc.robot.auto.reef.Branch.Side;
import frc.robot.auto.reef.Reef;
import frc.robot.auto.source.SourceChooser;
import frc.robot.auto.source.SourceChooser.SourceLocations;
import frc.robot.command_factories.ElevatorFactory;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.subsystems.climber.Climber;
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
import frc.robot.subsystems.remover.Remover;
import frc.robot.subsystems.remover.RemoverIO;
import frc.robot.subsystems.remover.RemoverIOSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.DashboardSwitch;
import frc.robot.util.PoseUtil;
import frc.robot.util.RoaringUtils.DeadzoneUtils;
import java.util.Set;
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
    private Drive drive; // Drive subsystem

    @SuppressWarnings("unused")
    private Vision vision; // Vision subsystem

    private Elevator elevator; // Elevator subsystem
    private Outtake outtake; // Outtake subsystem
    private Remover remover;
    private Climber climber = new Climber();
    private static boolean manualControls = false;

    private static Reef reef = new Reef(); // Reef object
    private static SourceChooser sourceChooser = new SourceChooser(); // Source chooser object
    private static SendableChooser<Level> heightChooser = new SendableChooser<>(); // Sendable chooser for selecting
    private static SendableChooser<Pose2d> arbPoseChooser = new SendableChooser<>();
    private static DashboardSwitch autoAlgaeSwitch = new DashboardSwitch(); // Switch for auto algae mode
    // height

    private static Command dynamicAutoBeta;
    private static Command dynamicAutoSingle;

    private static SwerveDriveSimulation driveSimulation = null; // Swerve drive simulation

    // Controller
    private final CommandXboxController controller; // Xbox controller
//     private final CommandGenericHID controller2; // Joystick

    // ButtonBoard buttonBoard = new ButtonBoard(reef); // Button board

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser; // Auto chooser

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Initialize Controller
        controller = new CommandXboxController(DRIVE_CONTROLLER); // Initialize Xbox controller
        // controller2 = new CommandGenericHID(BB_PORT);

        heightChooser.setDefaultOption("L4", Level.L4); // Set default height option
        heightChooser.addOption("L3", Level.L3); // Add L3 option
        heightChooser.addOption("L2", Level.L2); // Add L2 option
        heightChooser.addOption("L1", Level.L1); // Add L1 option
        SmartDashboard.putData("Height Chooser", heightChooser);

        switch (Constants.CURRENT_MODE) {
            case REAL: {
                // Real robot, instantiate hardware IO implementations
                drive = new Drive(
                        new GyroIOPigeon2(DriveConstants.PIGEON_CAN_ID),
                        new ModuleIOSpark(0),
                        new ModuleIOSpark(1),
                        new ModuleIOSpark(2),
                        new ModuleIOSpark(3)); // Initialize drive subsystem
                this.vision = new Vision(
                        drive,
                        new VisionIOPhotonVision(VisionConstants.CAMERA_0_NAME, VisionConstants.robotToCamera0),
                        new VisionIOPhotonVision(
                                VisionConstants.CAMERA_1_NAME, VisionConstants.robotToCamera1)); // Initialize vision
                // subsystem
                this.elevator = new Elevator(new ElevatorIOSpark(), this); // Initialize elevator
                // subsystem
                this.outtake = new Outtake(new OuttakeIOSpark(), elevator); // Initialize outtake
                // subsystem
                this.remover = new Remover(new RemoverIOSpark());
                break;
            }

            case SIM: {
                // create a maple-sim swerve drive simulation instance
                driveSimulation = new SwerveDriveSimulation(
                        DriveConstants.mapleSimConfig,
                        new Pose2d(3, 3, new Rotation2d())); // Initialize drive simulation
                // add the simulated drivetrain to the simulation field
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation); // Add drive
                // simulation to
                // arena
                // Sim robot, instantiate physics sim IO implementations
                drive = new Drive(
                        new GyroIOSim(driveSimulation.getGyroSimulation()),
                        new ModuleIOSim(driveSimulation.getModules()[0]),
                        new ModuleIOSim(driveSimulation.getModules()[1]),
                        new ModuleIOSim(driveSimulation.getModules()[2]),
                        new ModuleIOSim(driveSimulation.getModules()[3])); // Initialize drive
                // subsystem
                vision = new Vision(
                        drive,
                        new VisionIOPhotonVisionSim(
                                CAMERA_0_NAME, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
                        new VisionIOPhotonVisionSim(
                                CAMERA_1_NAME,
                                robotToCamera1,
                                driveSimulation::getSimulatedDriveTrainPose)); // Initialize
                // vision
                // subsystem

                this.elevator = new Elevator(new ElevatorIO() {}, this); // Initialize elevator subsystem
                this.outtake = new Outtake(new OuttakeIO() {}, elevator); // Initialize outtake subsystem
                this.remover = new Remover(new RemoverIO() {});

                break;
            }
            default: {
                // Replayed robot, disable IO implementations
                drive = new Drive(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {}); // Initialize drive subsystem
                vision = new Vision(drive, new VisionIO() {}, new VisionIO() {}); // Initialize vision subsystem
                this.elevator = new Elevator(new ElevatorIO() {}, this); // Initialize elevator subsystem
                this.outtake = new Outtake(new OuttakeIO() {}, elevator); // Initialize outtake subsystem
                this.remover = new Remover(new RemoverIO() {});

                break;
            }
        }
        // auto = new Autos(drive, outtake, elevator, remover);
        dynamicAutoBeta = Commands.sequence(
                // AutoBuilder.followPath(generatePath(
                //                 getPose(),
                //                 reef.getclosestBranch(AutoBuilder.getCurrentPose(), heightChooser.getSelected(),
                // false)
                //                         .getPose())
                //         .get()),
                Commands.defer(GoToReef(false, false), Set.of(drive)).withName("Auto Reef Align"),
                new PrintCommand("Reef aligned"),
                Commands.defer(ElevatorUp(), Set.of(elevator)).withName("Auto Elevator Up"),
                new PrintCommand("Elevator Up"),
                new WaitCommand(0.25),
                Commands.defer(() -> outtake.outtakeOutCmd(true), Set.of(outtake)),
                Commands.either(
                        Commands.defer(() -> remover.L3Algae(elevator, drive), Set.of(remover)),
                        new PrintCommand("No algae removal"),
                        autoAlgaeSwitch::get),
                new PrintCommand("Outtaked coral"),
                // Commands.defer(FillReefSlot(), autoReqs),
                // new PrintCommand("Filled Reef Slot"),
                ElevatorFactory.elevatorIntake(elevator),
                new PrintCommand("Elevator Down"),
                Commands.defer(GoToSource(), Set.of(drive)).withName("Auto Source Align"),
                new PrintCommand("Aligned to source"),
                outtake.outtakeInCmd(true),
                Commands.defer(GoToReef(true, false), Set.of(drive)).withName("Auto Reef Align"),
                // AutoBuilder.followPath(generatePath(
                //                 getPose(),
                //                 reef.getclosestBranch(AutoBuilder.getCurrentPose(), heightChooser.getSelected(),
                // false)
                //                         .getPose())
                //         .get()),
                new PrintCommand("Reef aligned"),
                Commands.defer(ElevatorUp(), Set.of(elevator)).withName("Auto Elevator Up"),
                new PrintCommand("Elevator Up"),
                new WaitCommand(0.75),
                Commands.defer(() -> outtake.outtakeOutCmd(true), Set.of(outtake)),
                new PrintCommand("Outtaked coral"),
                // Commands.defer(FillReefSlot(), autoReqs),
                // new PrintCommand("Filled Reef Slot"),
                ElevatorFactory.elevatorIntake(elevator));
        dynamicAutoSingle = Commands.sequence(
                Commands.defer(GoToReef(true, false), Set.of(drive)),
                new PrintCommand("Reef aligned"),
                Commands.defer(ElevatorUp(), Set.of(elevator)),
                new PrintCommand("Elevator Up"),
                new WaitCommand(2),
                Outtake(),
                new PrintCommand("Outtaked coral"),
                Commands.either(
                        Commands.defer(() -> remover.L3Algae(elevator, drive), Set.of(remover)),
                        new PrintCommand("No algae removal"),
                        autoAlgaeSwitch::get),
                Commands.defer(ElevatorDown(), Set.of(elevator)),
                new PrintCommand("Elevator Down"));
        // Initialize dynamic auto beta command

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices"); // Initialize
        // auto
        // chooser
        // if (Boolean.FALSE.equals(Constants.COMPETITION)) {
        // // Set up SysId routines
        // autoChooser.addOption(
        // "Drive Wheel Radius Characterization",
        // DriveCommands.wheelRadiusCharacterization(drive)); // Add wheel radius
        // // characterization option
        // autoChooser.addOption(
        // "Drive Simple FF Characterization",
        // DriveCommands.feedforwardCharacterization(drive)); // Add feedforward
        // // characterization option
        // autoChooser.addOption(
        // "Drive SysId (Quasistatic Forward)",
        // drive.sysIdQuasistatic(Sus.Direction.kForward)); // Add SysId
        // // quasistatic forward
        // // option
        // autoChooser.addOption(
        // "Drive SysId (Quasistatic Reverse)",
        // drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)); // Add SysId
        // // quasistatic reverse
        // // option
        // autoChooser.addOption(
        // "Drive SysId (Dynamic Forward)",
        // drive.sysIdDynamic(SysIdRoutine.Direction.kForward)); // Add SysId dynamic
        // // forward option
        // autoChooser.addOption(
        // "Drive SysId (Dynamic Reverse)",
        // drive.sysIdDynamic(SysIdRoutine.Direction.kReverse)); // Add SysId dynamic
        // // reverse option
        // }
        autoChooser.addDefaultOption(
                "2 Coral Auto", Commands.deferredProxy(() -> dynamicAutoBeta)); // Add dynamic auto option
        autoChooser.addOption("1 Coral Auto", dynamicAutoSingle);
        autoChooser.addOption(
                "Leave",
                Commands.sequence(
                        Commands.runOnce(() -> vision.enableUpdates(false)),
                        Commands.defer(
                                        () -> AutoBuilder.followPath(
                                                generatePathNoflip(getPose(), PoseUtil.offsetPose(getPose(), 1, 0))
                                                        .get()),
                                        Set.of(drive))
                                .finallyDo(() -> vision.enableUpdates(true)),
                        Commands.runOnce(() -> vision.enableUpdates(true))));
        // autoChooser.addOption("Dynamic Auto Beta", dynamicAutoBeta.repeatedly()); //
        // Add dynamic auto beta
        // option
        // Configure the button bindings

        arbPoseChooser.setDefaultOption("Center", new Pose2d(7.25, 4, Rotation2d.k180deg));
        arbPoseChooser.addOption("Left", new Pose2d(7.25, 6, Rotation2d.k180deg));
        arbPoseChooser.addOption("Right", new Pose2d(7.25, 2, Rotation2d.k180deg));
        SmartDashboard.putData("Backup side chooser", arbPoseChooser);
        SmartDashboard.putData("Auto Algae", autoAlgaeSwitch);
        configureButtonBindings(); // Configure button bindings
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by instantiating a
     * {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
     * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        NamedCommands.registerCommand("Elevator L4", ElevatorFactory.elevatorL4(elevator));
        NamedCommands.registerCommand("Elevator L3", ElevatorFactory.elevatorL3(elevator));
        NamedCommands.registerCommand("Elevator L2", ElevatorFactory.elevatorL2(elevator));
        NamedCommands.registerCommand("Elevator L1", ElevatorFactory.elevatorL1(elevator));
        NamedCommands.registerCommand("Elevator Intake", ElevatorFactory.elevatorIntake(elevator));

        NamedCommands.registerCommand("Intake", outtake.outtakeInCmd(true));
        NamedCommands.registerCommand("Outtake", outtake.outtakeOutCmd(true));

        // Default command, normal field-relative drive
        drive.setDefaultCommand(DriveCommands.joystickDrive(
                        drive,
                        () -> -DeadzoneUtils.LinearDeadband(controller.getLeftY(), 0.02),
                        () -> -DeadzoneUtils.LinearDeadband(controller.getLeftX(), 0.02),
                        () -> -DeadzoneUtils.LinearDeadband(controller.getRightX(), 0.02))
                .withName("Joystick Drive"));

        // Switch to X pattern when X button is pressed
        //controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

        // Reset gyro / odometry
        final Runnable resetGyro = Constants.CURRENT_MODE == Constants.Mode.SIM
                ? (() -> drive.resetOdometry(driveSimulation.getSimulatedDriveTrainPose())) // reset
                // odometry
                // to
                // actual robot pose
                // during simulation
                : (() -> drive.resetOdometry(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()))); // zero
        // gyro

        controller.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

        // controller.a().whileTrue(new RunCommand(() ->
        // DriveCommands.goToSource(sourceChooser)));
        // controller
        // .y()
        // .whileTrue(new RunCommand(() -> DriveCommands.goToReef(reef,
        // buttonBoard.getSelectedBranchSide())));
        controller.leftBumper().whileTrue(Commands.defer(GoToReef(Side.LEFT), Set.of(drive)));
        controller.rightBumper().whileTrue(Commands.defer(GoToReef(Side.RIGHT), Set.of(drive)));
        //controller.a().whileTrue(Commands.defer(GoToReef(false, false), Set.of(drive)));
        //controller.y().whileTrue(Commands.defer(GoToSource(), Set.of(drive)).withName("Auto Align Source"));

        // controller2.rightTrigger().whileTrue(outtake.outtakeOutCmd(!manualControls));
        controller.rightTrigger().whileTrue(Commands.defer(() -> outtake.outtakeOutCmd(!manualControls), Set.of(outtake)));
        // controller2.leftTrigger().whileTrue(outtake.outtakeInCmd(!manualControls));
        controller.leftTrigger().whileTrue(Commands.defer(() -> outtake.outtakeInCmd(!manualControls), Set.of(outtake)));

        controller.y().whileTrue(outtake.outtakeReverseCMD());

        // controller2.povUp().whileTrue(ElevatorFactory.elevator(elevator, Level.L4));
        controller2.povUp().or(controller2.povDown()).whileTrue(ElevatorFactory.elevatorL4(elevator));

        // controller2.povRight().whileTrue(ElevatorFactory.elevator(elevator,
        // Level.L2));
        controller2.povLeft().or(controller2.povRight()).whileTrue(ElevatorFactory.elevatorL2(elevator));

        // controller2.povDown().whileTrue(ElevatorFactory.elevator(elevator,
        // Level.L1));
        controller2.povUpLeft().or(controller2.povDownRight()).whileTrue(ElevatorFactory.elevatorL1(elevator));

        // controller2.povLeft().whileTrue(ElevatorFactory.elevator(elevator,
        // Level.L3));
        controller2.povUpRight().or(controller2.povDownLeft()).whileTrue(ElevatorFactory.elevatorL3(elevator));

        // controller2.a().whileTrue(ElevatorFactory.elevatorIntake(elevator));
        controller2.button(7).whileTrue(ElevatorFactory.elevatorIntake(elevator));

        // controller2.y().whileTrue(dynamicAutoBeta);

        // //
        // controller2.leftBumper().whileTrue(AlgaeArmFactory.AlgaeArmInside(algaeArm));
        // controller2
        // .leftBumper()
        // .whileTrue(AlgaeArmFactory.AlgaeArmIntake(algaeArm))
        // .onFalse(AlgaeArmFactory.AlgaeArmHold(algaeArm));
        // .onFalse(AlgaeArmFactory.AlgaeArmHold(algaeArm));

        // controller2
        // .rightBumper()
        // .whileTrue(AlgaeArmFactory.AlgaeArmRelease(algaeArm))
        // .onFalse(AlgaeArmFactory.AlgaeArmInside(algaeArm));

        controller2.button(1).debounce(1).onTrue(Commands.runOnce(() -> {
            manualControls = !manualControls;
        }));

        // if (!COMPETITION) {
        // controller.povUp().onTrue(ElevatorFactory.elevatorDynamicTest(elevator,
        // controller.povUp()));
        // controller.povDown().onTrue(ElevatorFactory.elevatorQuasistaticTest(elevator,
        // controller.povDown()));
        // }

        // controller2.b().whileTrue(remover.ArmOut());
        // controller2.x().whileTrue(remover.ArmIn());
        controller2
                .button(3)
                .onTrue(Commands.either(
                        remover.ArmOut(),
                        remover.L3Algae(elevator, drive).andThen(remover.ArmInAuto()),
                        () -> isManual()));
        controller2
                .button(5)
                .onTrue(Commands.either(
                        remover.ArmIn(),
                        remover.L2Algae(elevator, drive).andThen(remover.ArmInAuto()),
                        () -> isManual()));
        controller.povUp().whileTrue(climber.climb(0.5));
        controller.povDown().whileTrue(climber.climb(-0.5));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    Pose2d pose = arbPoseChooser.getSelected();
                    if (Robot.isRedAlliance()) {
                        pose = FlippingUtil.flipFieldPose(pose);
                    }
                    resetPose(pose);
                }),
                autoChooser.get());
    }

    public void resetSimulationField() {
        Pose2d startPose = new Pose2d(7.393, 6.572, Rotation2d.fromDegrees(-140));
        if (Constants.CURRENT_MODE != Constants.Mode.SIM) return;

        driveSimulation.setSimulationWorldPose(startPose);
        SimulatedArena.getInstance().resetFieldForAuto();

        drive.resetOdometry(startPose);
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

    public Supplier<Command> GoToReef(Boolean useVision, Boolean targetSource) {
        return () -> AutoBuilder.pathfindThenFollowPath(
                        generatePath(
                                        PoseUtil.offsetPose(
                                                reef.getclosestBranch(
                                                                (targetSource
                                                                        ? sourceChooser.getClosestSourcePose()
                                                                        : AutoBuilder.getCurrentPose()),
                                                                heightChooser.getSelected(),
                                                                useVision)
                                                        .getPose(),
                                                -Units.feetToMeters(1),
                                                0),
                                        PoseUtil.offsetPose(
                                                reef.getclosestBranch(
                                                                (targetSource
                                                                        ? sourceChooser.getClosestSourcePose()
                                                                        : AutoBuilder.getCurrentPose()),
                                                                heightChooser.getSelected(),
                                                                useVision)
                                                        .getPose(),
                                                -Units.inchesToMeters(0),
                                                0))
                                .get(),
                        FINDINGCONSTRAINTS)
                .finallyDo(() -> drive.stop());
    }

    /**
     * Generates a path from a starting pose to an ending pose using PathPlanner.
     *
     * @param start The starting pose of the path.
     * @param end The ending pose of the path.
     * @return A Supplier that generates a PathPlannerPath with the calculated waypoints and constraints.
     */
    public static Supplier<PathPlannerPath> generatePath(Pose2d start, Pose2d end) {
        // Calculate the starting rotation based on the direction from start to end
        Rotation2d startRotation = new Rotation2d(Math.atan2(end.getY() - start.getY(), end.getX() - start.getX()));

        // Use the rotation of the end pose as the end rotation
        Rotation2d endRotation = end.getRotation();

        // Return a supplier that generates a PathPlannerPath with the calculated
        // waypoints and constraints
        return () -> new PathPlannerPath(
                // Create waypoints from the start and end poses with the calculated rotations
                PathPlannerPath.waypointsFromPoses(
                        new Pose2d(start.getTranslation(), startRotation),
                        new Pose2d(end.getTranslation(), endRotation)),
                // Use predefined path constraints
                PATHCONSTRAINTS,
                // Define the ideal starting state with a velocity of 0.5 and the calculated end
                // rotation
                new IdealStartingState(0.5, endRotation),
                // Define the goal end state with a velocity of 0.0 and the calculated end
                // rotation
                new GoalEndState(0.0, endRotation));
    }

    /**
     * Generates a path from a starting pose to an ending pose using PathPlanner.
     *
     * @param start The starting pose of the path.
     * @param end The ending pose of the path.
     * @return A Supplier that generates a PathPlannerPath with the calculated waypoints and constraints.
     */
    public static Supplier<PathPlannerPath> generatePathNoflip(Pose2d start, Pose2d end) {
        // Calculate the starting rotation based on the direction from start to end
        Rotation2d startRotation = new Rotation2d(Math.atan2(end.getY() - start.getY(), end.getX() - start.getX()));

        // Use the rotation of the end pose as the end rotation
        Rotation2d endRotation = end.getRotation();

        // Return a supplier that generates a PathPlannerPath with the calculated
        // waypoints and constraints
        PathPlannerPath path = new PathPlannerPath(
                // Create waypoints from the start and end poses with the calculated rotations
                PathPlannerPath.waypointsFromPoses(
                        new Pose2d(start.getTranslation(), startRotation),
                        new Pose2d(end.getTranslation(), endRotation)),
                // Use predefined path constraints
                PATHCONSTRAINTS,
                // Define the ideal starting state with a velocity of 0.5 and the calculated end
                // rotation
                new IdealStartingState(0.5, endRotation),
                // Define the goal end state with a velocity of 0.0 and the calculated end
                // rotation
                new GoalEndState(0.0, endRotation));

        path.preventFlipping = true;
        return () -> path;
    }

    public static Supplier<Command> GoToSource() {

        return () -> AutoBuilder.pathfindThenFollowPath(
                generatePath(
                                PoseUtil.offsetPose(sourceChooser.getClosestSourcePose(), Units.feetToMeters(1), 0),
                                PoseUtil.offsetPose(sourceChooser.getClosestSourcePose(), Units.inchesToMeters(0), 0))
                        .get(),
                FINDINGCONSTRAINTS);
    }

    public Supplier<Command> GoToSource(Side side) {

        return () -> AutoBuilder.pathfindThenFollowPath(
                        generatePath(
                                        PoseUtil.offsetPose(
                                                (side == Side.RIGHT)
                                                        ? SourceLocations.SOURCE_RIGHT
                                                        : SourceLocations.SOURCE_LEFT,
                                                Units.feetToMeters(1),
                                                0),
                                        PoseUtil.offsetPose(
                                                (side == Side.RIGHT)
                                                        ? SourceLocations.SOURCE_RIGHT
                                                        : SourceLocations.SOURCE_LEFT,
                                                Units.inchesToMeters(0), // 6
                                                0))
                                .get(),
                        FINDINGCONSTRAINTS)
                .finallyDo(() -> drive.stop());
    }

    public void transition() {
        elevator.setElevatorHeight(0);
    }

    public Supplier<Command> ElevatorUp() {
        return () -> {
            Level currentLevel = PRIORITY_LEVEL; // Set current level to priority level
            if (!reef.getclosestBranch(AutoBuilder.getCurrentPose(), PRIORITY_LEVEL, true)
                    .getCoralStatus(PRIORITY_LEVEL)) { // Check if coral status is false
                return ElevatorFactory.elevator(elevator, PRIORITY_LEVEL); // Run elevator command
            }
            while (reef.getclosestBranch(AutoBuilder.getCurrentPose(), PRIORITY_LEVEL, true)
                    .getCoralStatus(currentLevel)) { // Loop to find free level
                currentLevel = Reef.getLesserLevel(currentLevel); // Get lesser level
            }
            Level freeLevel = currentLevel; // Set free level
            return ElevatorFactory.elevator(elevator, freeLevel); // Run elevator command
        };
    }

    public Command Outtake() {
        return outtake.outtakeOutCmd(true);
    }

    public Supplier<Command> ElevatorDown() {
        return () -> ElevatorFactory.elevatorIntake(elevator); // Run elevator intake command
    }

    public Supplier<Command> Intake() {
        return () -> outtake.outtakeInCmd(true); // Run intake command
    }

    public Supplier<Command> FillReefSlot() {
        return () -> Commands.run(() -> {
            Level currentLevel = PRIORITY_LEVEL; // Set current level to priority level
            if (!reef.getclosestBranch(AutoBuilder.getCurrentPose(), currentLevel, true)
                    .getCoralStatus(PRIORITY_LEVEL)) { // Check if coral status is false
                reef.getclosestBranch(AutoBuilder.getCurrentPose(), PRIORITY_LEVEL, true)
                        .setCoralStatus(PRIORITY_LEVEL, true); // Set coral status to true
            } else {
                while (reef.getclosestBranch(AutoBuilder.getCurrentPose(), currentLevel, true)
                        .getCoralStatus(currentLevel)) { // Loop to find free level
                    currentLevel = Reef.getLesserLevel(currentLevel); // Get lesser level
                }
                reef.getclosestBranch(AutoBuilder.getCurrentPose(), currentLevel, true)
                        .setCoralStatus(currentLevel, true); // Set coral status to true
            }
        });
    }

    public boolean hasCoral() {
        return outtake.isLoaded();
    }

    public static boolean isManual() {
        Boolean manual = manualControls;
        return manual;
    }

    public void enableVisionUpdates(boolean enable) {
        vision.enableUpdates(enable);
    }

    public Supplier<Command> GoToReef(Side side) {
        return () -> AutoBuilder.pathfindThenFollowPath(
                        generatePath(
                                        PoseUtil.offsetPose(
                                                reef.getclosestFace(AutoBuilder.getCurrentPose())
                                                        .getBranch(side)
                                                        .getPose(),
                                                -Units.feetToMeters(1),
                                                0),
                                        PoseUtil.offsetPose(
                                                reef.getclosestFace(AutoBuilder.getCurrentPose())
                                                        .getBranch(side)
                                                        .getPose(),
                                                -Units.inchesToMeters(0),
                                                0))
                                .get(),
                        FINDINGCONSTRAINTS)
                .finallyDo(() -> drive.stop());
    }
}
