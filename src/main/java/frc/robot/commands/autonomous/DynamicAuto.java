// package frc.robot.commands.autonomous;

// import com.pathplanner.lib.auto.AutoBuilder;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.RobotContainer;
// import frc.robot.auto.reef.Branch.Level;
// import frc.robot.auto.reef.Reef;
// import frc.robot.auto.source.SourceChooser;
// import frc.robot.subsystems.drive.Drive;

// /**
//  * DynamicAuto is a command that dynamically schedules paths to either a reef or a source based on the current state
// of
//  * the reef and the robot's position.
//  *
//  * <p>This command alternates between going to the reef and going to the source, updating the reef's coral status as
// it
//  * progresses.
//  *
//  * @param reef The reef object representing the target reef.
//  * @param sourceChooser The source chooser object for selecting the source.
//  * @param drive The drive subsystem required by this command.
//  */
// public class DynamicAuto extends Command {
//     private final Reef reef; // The reef object to interact with

//     private Command currentCommand; // The current command being executed
//     private boolean goingToReef = true; // Flag to determine if the robot is going to the reef
//     private boolean isDone = false; // Flag to determine if the task is complete
//     Pose2d currentPose; // The current position of the robot

//     /**
//      * Constructor for DynamicAuto.
//      *
//      * @param reef The reef object representing the target reef.
//      * @param chooser The source chooser object for selecting the source.
//      * @param drive The drive subsystem required by this command.
//      */
//     public DynamicAuto(Reef reef, SourceChooser chooser, Drive drive) {
//         this.reef = reef;
//         addRequirements(drive); // Add drive subsystem as a requirement
//     }

//     /** Initializes the command by getting the current pose and scheduling the next path. */
//     @Override
//     public void initialize() {
//         currentPose = AutoBuilder.getCurrentPose(); // Get the current pose of the robot
//         System.out.println("[DynamicAutoV2] Starting...");
//         scheduleNextPath(); // Schedule the next path
//     }

//     /** Executes the command by checking if the current command is running and scheduling the next path if not. */
//     @Override
//     public void execute() {
//         currentPose = AutoBuilder.getCurrentPose(); // Update the current pose of the robot
//         if (currentCommand == null || !currentCommand.isScheduled()) { // Check if the current command is not running
//             System.out.println("[DynamicAutoV2] Current command is not running. Scheduling next path...");
//             scheduleNextPath(); // Schedule the next path
//         }
//     }

//     /** Schedules the next path based on the current state and position. */
//     private void scheduleNextPath() {
//         if (!goingToReef) { // If not going to the reef, update the coral status
//             if (!reef.getclosestBranch(currentPose, Level.L3, true).getCoralStatus(Level.L3)) {
//                 reef.getclosestBranch(currentPose, Level.L3, true).setCoralStatus(Level.L3, true);
//             } else if (!reef.getclosestBranch(currentPose, Level.L3, true).getCoralStatus(Level.L2)) {
//                 reef.getclosestBranch(currentPose, Level.L3, true).setCoralStatus(Level.L2, true);
//             } else if (!reef.getclosestBranch(currentPose, Level.L3, true).getCoralStatus(Level.L1)) {
//                 reef.getclosestBranch(currentPose, Level.L3, true).setCoralStatus(Level.L1, true);
//             } else if (reef.getclosestBranch(currentPose, Level.L3, true).getCoralStatus(Level.L4)) {
//                 reef.getclosestBranch(currentPose, Level.L3, true).setCoralStatus(Level.L4, true);
//             }
//         }

//         System.out.println("[DynamicAutoV2] Scheduling path to " + (goingToReef ? "REEF" : "SOURCE"));

//         if (currentCommand != null) { // If there is a current command, cancel it
//             System.out.println("[DynamicAutoV2] Cancelling previous command...");
//             currentCommand.cancel();
//         }

//         // Schedule the next command based on the current state (going to reef or source)
//         currentCommand = ((goingToReef)
//                         ? RobotContainer.GoToReef(true, true).get()
//                         : RobotContainer.GoToSource().get())
//                 .andThen(() -> {
//                     System.out.println("[DynamicAutoV2] Finished path to " + (goingToReef ? "REEF" : "SOURCE"));
//                     goingToReef = !goingToReef; // Toggle the state after completion
//                     if (currentCommand != null) {
//                         currentCommand.cancel();
//                     }
//                     currentPose = AutoBuilder.getCurrentPose(); // Update the current pose
//                     scheduleNextPath(); // Schedule the next path
//                 });

//         currentCommand.schedule(); // Schedule the current command
//         if (reef.isReefFull()) { // Check if the reef is full
//             isDone = true;
//             System.out.println("[DynamicAutoV2] Reef is full.");
//         }

//         System.out.println("[DynamicAutoV2] Path to " + (goingToReef ? "REEF" : "SOURCE") + " started.");
//     }

//     /**
//      * Ends the command, cancelling the current command if necessary.
//      *
//      * @param interrupted Whether the command was interrupted.
//      */
//     @Override
//     public void end(boolean interrupted) {
//         if (currentCommand != null) { // If there is a current command, cancel it
//             currentCommand.cancel();
//         }
//         System.out.println("[DynamicAutoV2] Command Ended. Interrupted? " + interrupted);
//         if (!isDone && reef.isReefFull()) { // Check if the reef is full and the task is not done
//             isDone = true;
//             System.out.println("[DynamicAutoV2] Reef is full. Ending command.");
//         }
//     }

//     /**
//      * Determines if the command is finished.
//      *
//      * @return True if the reef is full, false otherwise.
//      */
//     @Override
//     public boolean isFinished() {
//         return reef.isReefFull(); // Runs until reef is full
//     }
// }
