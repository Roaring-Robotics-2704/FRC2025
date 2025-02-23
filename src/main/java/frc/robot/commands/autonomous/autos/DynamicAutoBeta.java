package frc.robot.commands.autonomous.autos;

import static frc.robot.subsystems.drive.DriveConstants.FINDINGCONSTRAINTS;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.auto.reef.Branch.Level;
import frc.robot.auto.reef.Reef;
import frc.robot.auto.source.SourceChooser;
import frc.robot.subsystems.drive.Drive;
import java.util.List;
import java.util.function.Supplier;

public class DynamicAutoBeta extends Command {

    private Reef reef;
    private SourceChooser sourceChooser;
    private int currentIndex = 0;

    private Command currentCommand;
    List<Supplier<Command>> commandList = List.of(goToReef(RobotContainer.getBluePose(), Level.L3), goToSource());

    public DynamicAutoBeta(Reef reef, SourceChooser chooser, Drive drive) {
        this.reef = reef;
        this.sourceChooser = chooser;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        System.out.println("[DynamicAutoV2] Starting...");
        scheduleNextCommand();
    }

    @Override
    public void execute() {
        if (currentCommand == null || !currentCommand.isScheduled()) {
            System.out.println("[DynamicAutoV2] Current command is not running. Scheduling next command...");
            scheduleNextCommand();
        }
    }

    private void scheduleNextCommand() {
        if (commandList.isEmpty()) {
            return;
        }

        System.out.println("[DynamicAutoV2] Scheduling command index " + currentIndex);

        if (currentCommand != null) {
            System.out.println("[DynamicAutoV2] Cancelling previous command...");
            currentCommand.cancel();
        }

        currentCommand = commandList.get(currentIndex).get().andThen(() -> {
            System.out.println("[DynamicAutoV2] Finished command index " + currentIndex);
            currentIndex = (currentIndex + 1) % commandList.size(); // Move to next command and loop
            scheduleNextCommand();
        });

        currentCommand.schedule();
        System.out.println("[DynamicAutoV2] Command index " + currentIndex + " started.");
    }

    @Override
    public void end(boolean interrupted) {
        if (currentCommand != null) {
            currentCommand.cancel();
        }
        System.out.println("[DynamicAutoV2] Command Ended. Interrupted? " + interrupted);
    }

    @Override
    public boolean isFinished() {
        return reef.isReefFull(); // Stop if the reef is full
    }

    private Command goToPose(Pose2d pose) {
        if (AutoBuilder.shouldFlip()) {
            return AutoBuilder.pathfindToPoseFlipped(pose, FINDINGCONSTRAINTS);
        } else {
            return AutoBuilder.pathfindToPose(pose, FINDINGCONSTRAINTS);
        }
    }

    private Supplier<Command> goToReef(Pose2d Bluepose, Level level) {
        return ()->goToPose(reef.getclosestBranch(Bluepose, level).getPose());
    }

    private Supplier<Command> goToSource() {
        return ()->goToPose(sourceChooser.getClosestSourcePose());
    }
}
