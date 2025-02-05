// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import static frc.robot.subsystems.drive.DriveConstants.CONSTRAINTS;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class ChasePose extends Command {
    private Drive swerve;
    private Supplier<Pose2d> desiredPoseSupplier;
    private BooleanSupplier cancelSupplier;

    /** Creates a new ChasePose. */
    public ChasePose(Drive swerve, Supplier<Pose2d> poseSupplier, BooleanSupplier cancelSupplier) {
        this.swerve = swerve;
        this.desiredPoseSupplier = poseSupplier;
        this.cancelSupplier = cancelSupplier;
        addRequirements(swerve);
    }

    /** Creates a new ChasePose. */
    public ChasePose(Drive swerve, Supplier<Pose2d> poseSupplier) {
        this(swerve, poseSupplier, () -> false);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        AutoBuilder.resetOdom(swerve.getPose());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        AutoBuilder.pathfindToPose(desiredPoseSupplier.get(), CONSTRAINTS);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return cancelSupplier.getAsBoolean(); // TODO add a cancel if robot is at desired pose
    }
}
