package frc.robot.commands.Autonomus;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import java.util.List;
import frc.robot.util.PoseUtil;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PathBuilder extends Command {
    Drive drive;
    Pose2d oldReefPose = new Pose2d();
    Pose2d oldSourcePose = new Pose2d();
    private List<Waypoint> toReefWaypoints;
    private List<Waypoint> toSourceWaypoints;
    PathPlannerPath toReefPath;
    PathPlannerPath toSourcePath;

    /** Creates a new ReefToAmp. */
    public PathBuilder(Drive drive) {
        addRequirements(drive);
        this.drive = drive;
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        oldReefPose = RobotContainer.getReefPose();
        oldSourcePose = RobotContainer.getSourcePose();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if ((oldSourcePose != RobotContainer.getSourcePose()) || (RobotContainer.getReefPose() != oldReefPose)) {
            oldReefPose = RobotContainer.getReefPose();
            oldSourcePose = RobotContainer.getSourcePose();
            toReefWaypoints = PathPlannerPath.waypointsFromPoses(oldSourcePose, oldReefPose);
            toSourceWaypoints = PathPlannerPath.waypointsFromPoses(oldReefPose, oldSourcePose);

            toReefPath = new PathPlannerPath(
                    toReefWaypoints,
                    DriveConstants.CONSTRAINTS,
                    null,
                    new GoalEndState(0, RobotContainer.getReefPose().getRotation()));
            toSourcePath = new PathPlannerPath(
                    toSourceWaypoints,
                    DriveConstants.CONSTRAINTS,
                    null,
                    new GoalEndState(0, RobotContainer.getSourcePose().getRotation()));
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        /* TODO document why this method is empty */
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
