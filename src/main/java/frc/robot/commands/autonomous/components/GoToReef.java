// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.components;

import static frc.robot.subsystems.drive.DriveConstants.CONSTRAINTS;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.reef.Branch.Level;
import frc.robot.auto.reef.Reef;
import frc.robot.subsystems.drive.Drive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToReef extends Command {
    private static final double POSITION_TOLERANCE = 0.1; // Example tolerance value
    private static final double ANGLE_TOLERANCE = 5.0; // Example tolerance value in degrees

    Pose2d reefPose;
    Drive drive;
    Reef reef;
    /** Creates a new DynamicAuto. */
    public GoToReef(Drive drive, Reef reef) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.drive = drive;
        this.reef = reef;
        addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        reefPose = reef.getclosestPose(drive.getPose(), Level.L3);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        reefPose = reef.getclosestPose(drive.getPose(), Level.L3);

        AutoBuilder.pathfindToPose(reefPose, CONSTRAINTS, 0).andThen(new WaitCommand(0.5));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        Pose2d currentPose = drive.getPose();
        double positionError = currentPose.getTranslation().getDistance(reefPose.getTranslation());
        double angleError = Math.abs(
                currentPose.getRotation().getDegrees() - reefPose.getRotation().getDegrees());

        return positionError <= POSITION_TOLERANCE && angleError <= ANGLE_TOLERANCE;
    }
}
