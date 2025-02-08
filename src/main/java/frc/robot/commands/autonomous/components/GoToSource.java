// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.components;

import static frc.robot.subsystems.drive.DriveConstants.CONSTRAINTS;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToSource extends Command {
    private static final double POSITION_TOLERANCE = Units.inchesToMeters(2); // tolerance value
    private static final double ANGLE_TOLERANCE = 5.0; // Example tolerance value in degrees
    SendableChooser<Pose2d> sourceChooser;
    Drive drive;
    Pose2d sourcePose;
    /** Creates a new GoToSource. */
    public GoToSource(SendableChooser<Pose2d> sourceChooser, Drive drive) {
        this.sourceChooser = sourceChooser;
        this.drive = drive;
        addRequirements(drive);

        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        sourcePose = sourceChooser.getSelected();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        sourcePose = sourceChooser.getSelected();
        AutoBuilder.pathfindToPose(sourcePose, CONSTRAINTS, 0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        Pose2d currentPose = drive.getPose();
        double positionError = currentPose.getTranslation().getDistance(sourcePose.getTranslation());
        double angleError = Math.abs(currentPose.getRotation().getDegrees()
                - sourcePose.getRotation().getDegrees());

        return positionError <= POSITION_TOLERANCE && angleError <= ANGLE_TOLERANCE;
    }
}
