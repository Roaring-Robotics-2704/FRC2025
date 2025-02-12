package frc.robot.commands.autonomous.autos;

import static frc.robot.subsystems.drive.DriveConstants.CONSTRAINTS;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.reef.Branch.Level;
import frc.robot.auto.reef.Reef;
import frc.robot.auto.source.SourceChooser;

public class DynamicAuto extends Command {
    private final Reef reef;
    private final SourceChooser sourceChooser;

    private Command currentCommand;
    private boolean goingToReef = true;

    private static final double SWITCH_THRESHOLD = Units.inchesToMeters(10); // Distance to trigger swap

    public DynamicAuto(Reef reef, SourceChooser chooser) {
        this.reef = reef;
        this.sourceChooser = chooser;
    }

    @Override
    public void initialize() {
        System.out.println("[DynamicAutoV2] Starting...");
        scheduleNextPath();
    }

    @Override
    public void execute() {
        if (currentCommand == null || !currentCommand.isScheduled()) {
            System.out.println("[DynamicAutoV2] Current command is not running. Scheduling next path...");
            scheduleNextPath();
        }
    }

    private void scheduleNextPath() {
        Pose2d currentPose = AutoBuilder.getCurrentPose();

        Pose2d targetPose =
                goingToReef ? reef.getclosestBranch(currentPose, Level.L3).getPose() : sourceChooser.getSourcePose();
        if (!goingToReef) {
            reef.getclosestBranch(currentPose, Level.L3).setCoralStatus(Level.L3, true);
        }

        System.out.println("[DynamicAutoV2] Scheduling path to " + (goingToReef ? "REEF" : "SOURCE"));

        if (currentCommand != null) {
            System.out.println("[DynamicAutoV2] Cancelling previous command...");
            currentCommand.cancel();
        }

        currentCommand = AutoBuilder.pathfindToPose(targetPose, CONSTRAINTS).andThen(() -> {
            System.out.println("[DynamicAutoV2] Finished path to " + (goingToReef ? "REEF" : "SOURCE"));
            goingToReef = !goingToReef; // Toggle AFTER completion
        });

        currentCommand.schedule();

        System.out.println("[DynamicAutoV2] Path to " + (goingToReef ? "REEF" : "SOURCE") + " started.");
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
        return false; // Runs indefinitely
    }
}
