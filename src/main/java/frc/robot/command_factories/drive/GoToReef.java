// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command_factories.drive;

import static frc.robot.subsystems.drive.DriveConstants.FINDINGCONSTRAINTS;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.reef.Branch.Level;
import frc.robot.auto.reef.Reef;
import frc.robot.subsystems.drive.Drive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToReef extends Command {
    Reef reef;
    Trigger button;
    CommandXboxController xbox;

    /** Creates a new GoToReef. */
    public GoToReef(Reef reef, Drive drive, CommandXboxController xbox) {
        this.reef = reef;
        this.xbox = xbox;
        button = xbox.a();
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        System.out.println("Reef command running");
        if (button.getAsBoolean()) {
            AutoBuilder.pathfindToPose(
                            reef.getclosestBranch(AutoBuilder.getCurrentPose(), Level.L3)
                                    .getPose(),
                            FINDINGCONSTRAINTS)
                    .execute();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    private Command reefCommand(Pose2d pose) {
        return AutoBuilder.pathfindToPose(reef.getclosestBranch(pose, Level.L3).getPose(), FINDINGCONSTRAINTS);
    }
}
