package frc.robot.commands.autonomous.autos;

import static frc.robot.subsystems.drive.DriveConstants.FINDINGCONSTRAINTS;
import static frc.robot.Constants.PRIORITY_LEVEL;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.auto.reef.Branch;
import frc.robot.auto.reef.Branch.Level;
import frc.robot.auto.reef.Reef;
import frc.robot.auto.source.SourceChooser;
import frc.robot.command_factories.ElevatorFactory;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.outtake.Outtake;

import java.util.List;
import java.util.function.Supplier;

public class DynamicAutoBeta extends Command {

    private Reef reef;
    private SourceChooser sourceChooser;
    private Elevator elevator;
    private Outtake outtake;

    private int currentIndex = 0;

    private Command currentCommand;
    List<Supplier<Command>> commandList;

    public DynamicAutoBeta(Reef reef, SourceChooser chooser, Drive drive, Elevator elevator, Outtake outtake) {
        this.reef = reef;
        this.sourceChooser = chooser;
        this.elevator = elevator;
        this.outtake = outtake;


        commandList = List.of(
                goToReef(),
                elevatorUp(),
                outtake(),
                elevatorDown(),
                goToSource(),
                intake()
                );

        addRequirements(drive, elevator);
    }

    @Override
    public void initialize() {
        System.out.println("[DynamicAutoV3] Starting...");
        // Start the first command in the sequence
        scheduleNextCommand();
    }

    @Override
    public void execute() {
        // Start next command if current command is not running
        if (currentCommand == null || !currentCommand.isScheduled()) {
            System.out.println("[DynamicAutoV3] Current command is not running. Scheduling next command...");
            scheduleNextCommand();
        }
    }

    /**
     * Schedules the next command in the sequence. Cancels the current command if it
     * is running.
     */
    private void scheduleNextCommand() {
        // If the command list is empty, there is nothing to schedule
        if (commandList.isEmpty()) {
            return;
        }

        // Log the scheduling of the next command
        System.out.println("[DynamicAutoV3] Scheduling command index " + currentIndex);

        // If there is a current command running, cancel it
        if (currentCommand != null) {
            System.out.println("[DynamicAutoV3] Cancelling previous command...");
            currentCommand.cancel();
        }

        // Get the next command from the list and schedule it
        currentCommand = commandList.get(currentIndex).get().andThen(() -> {
            // Log the completion of the current command
            System.out.println("[DynamicAutoV2] Finished command index " + currentIndex);
            // Move to the next command in the list, looping back to the start if necessary
            currentIndex = (currentIndex + 1) % commandList.size();
            // Schedule the next command
            scheduleNextCommand();
        });

        // Schedule the current command
        currentCommand.schedule();
        // Log the start of the current command
        System.out.println("[DynamicAutoV3] Command index " + currentIndex + " started.");
    }

    @Override
    public void end(boolean interrupted) {
        // Cancel the current command if it is running
        if (currentCommand != null) {
            currentCommand.cancel();
        }
        // Log the end of the command
        System.out.println("[DynamicAutoV3] Command Ended. Interrupted? " + interrupted);
    }

    @Override
    public boolean isFinished() {
        return reef.isReefFull(); // Stop if the reef is full
    }

    private Command goToPose(Pose2d pose) {
        if (AutoBuilder.shouldFlip()) { // Flip the path if we are on the red side
            return AutoBuilder.pathfindToPoseFlipped(pose, FINDINGCONSTRAINTS); // Go to flipped pose
        } else {
            return AutoBuilder.pathfindToPose(pose, FINDINGCONSTRAINTS); // Go to pose
        }
    }

    private Supplier<Command> goToReef() {
        return () -> goToPose(reef.getclosestBranch(RobotContainer.getBluePose(), PRIORITY_LEVEL).getPose()); // Get the
                                                                                                              // closest
                                                                                                              // branch
                                                                                                              // on the
                                                                                                              // reef
        // and go to it
    }

    private Supplier<Command> goToSource() {
        return () -> goToPose(sourceChooser.getClosestSourcePose()); // Get the closest source pose and go to it
    }

    @SuppressWarnings("static-access")
    private Supplier<Command> elevatorUp() {
        Level currentLevel = PRIORITY_LEVEL;
        Branch branch = reef.getclosestBranch(AutoBuilder.getCurrentPose(), PRIORITY_LEVEL);
        if (branch.getCoralStatus(PRIORITY_LEVEL)) {
            return () -> ElevatorFactory.elevator(elevator, PRIORITY_LEVEL); // Go to the Priority level if 
        } else {
            while (!branch.getCoralStatus(currentLevel)) {
                currentLevel = reef.getLesserLevel(currentLevel);
            }
            Level elevatorLevel = currentLevel;
            return () -> ElevatorFactory.elevator(elevator, elevatorLevel); // Go to the current level
        }
    }
    private Supplier<Command> elevatorDown() {
        return () -> ElevatorFactory.elevatorIntake(elevator); // Go to the intake height
    }

    private Supplier<Command> outtake() {
        return () -> outtake.outtakeOutCmd(); // Outtake the coral
    }
    private Supplier<Command> intake() {
        return () -> outtake.outtakeInCmd(); // Outtake the coral
    }
}
