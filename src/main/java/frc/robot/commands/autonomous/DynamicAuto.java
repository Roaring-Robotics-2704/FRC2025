package frc.robot.commands.autonomous;

import static frc.robot.subsystems.drive.DriveConstants.FINDINGCONSTRAINTS;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.auto.reef.Branch.Level;
import frc.robot.auto.reef.Reef;
import frc.robot.auto.source.SourceChooser;
import frc.robot.subsystems.drive.Drive;

public class DynamicAuto extends Command {
    private final Reef reef;
    private final SourceChooser sourceChooser;

    private Command currentCommand;
    private boolean goingToReef = true;
    private boolean isDone = false;
    Pose2d currentPose;

    public DynamicAuto(Reef reef, SourceChooser chooser, Drive drive) {
        this.reef = reef;
        this.sourceChooser = chooser;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        currentPose = AutoBuilder.getCurrentPose();
        System.out.println("[DynamicAutoV2] Starting...");
        scheduleNextPath();
    }

    @Override
    public void execute() {
        currentPose = AutoBuilder.getCurrentPose();
        if (currentCommand == null || !currentCommand.isScheduled()) {
            System.out.println("[DynamicAutoV2] Current command is not running. Scheduling next path...");
            scheduleNextPath();
        }
    }

    private void scheduleNextPath() {
        currentPose = AutoBuilder.getCurrentPose();

        Pose2d targetPose =
                goingToReef ? reef.getclosestBranch(currentPose, Level.L3).getPose() : sourceChooser.getSourcePose();
        if (!goingToReef) {
            if (!reef.getclosestBranch(currentPose, Level.L3).getCoralStatus(Level.L3)) {
                reef.getclosestBranch(currentPose, Level.L3).setCoralStatus(Level.L3, true);

            } else if (!reef.getclosestBranch(currentPose, Level.L3).getCoralStatus(Level.L2)) {
                reef.getclosestBranch(currentPose, Level.L3).setCoralStatus(Level.L2, true);
            } else if (!reef.getclosestBranch(currentPose, Level.L3).getCoralStatus(Level.L1)) {
                reef.getclosestBranch(currentPose, Level.L3).setCoralStatus(Level.L1, true);
            } else if (reef.getclosestBranch(currentPose, Level.L3).getCoralStatus(Level.L4)) {
                reef.getclosestBranch(currentPose, Level.L3).setCoralStatus(Level.L4, true);
            }
        }

        System.out.println("[DynamicAutoV2] Scheduling path to " + (goingToReef ? "REEF" : "SOURCE"));

        if (currentCommand != null) {
            System.out.println("[DynamicAutoV2] Cancelling previous command...");
            currentCommand.cancel();
        }

        if (Robot.isRedAlliance()) {
            currentCommand = AutoBuilder.pathfindToPoseFlipped(targetPose, FINDINGCONSTRAINTS)
                    .andThen(() -> {
                        System.out.println("[DynamicAutoV2] Finished path to " + (goingToReef ? "REEF" : "SOURCE"));
                        goingToReef = !goingToReef; // Toggle AFTER completion
                        if (currentCommand != null) {
                            currentCommand.cancel();
                        }
                        scheduleNextPath();
                    });
        } else {
            currentCommand = AutoBuilder.pathfindToPose(targetPose, FINDINGCONSTRAINTS)
                    .andThen(() -> {
                        System.out.println("[DynamicAutoV2] Finished path to " + (goingToReef ? "REEF" : "SOURCE"));
                        goingToReef = !goingToReef; // Toggle AFTER completion
                        if (currentCommand != null) {
                            currentCommand.cancel();
                        }
                        currentPose = AutoBuilder.getCurrentPose();
                        scheduleNextPath();
                    });
        }

        currentCommand.schedule();
        if (reef.isReefFull()) {
            isDone = true;
            System.out.println("[DynamicAutoV2] Reef is full.");
        }

        System.out.println("[DynamicAutoV2] Path to " + (goingToReef ? "REEF" : "SOURCE") + " started.");
    }

    @Override
    public void end(boolean interrupted) {
        if (currentCommand != null) {
            currentCommand.cancel();
        }
        System.out.println("[DynamicAutoV2] Command Ended. Interrupted? " + interrupted);
        if (!isDone && reef.isReefFull()) {
            isDone = true;
            System.out.println("[DynamicAutoV2] Reef is full. Ending command.");
        }
    }

    @Override
    public boolean isFinished() {

        return reef.isReefFull(); // Runs until reef is full
    }
}
