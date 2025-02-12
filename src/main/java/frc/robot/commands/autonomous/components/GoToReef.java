// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.components;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.reef.Branch.Level;
import frc.robot.RobotContainer;
import frc.robot.auto.reef.Reef;
import frc.robot.commands.drive.DriveCommands;

/** Add your docs here. */
public class GoToReef {
    Reef reef;
    public static Supplier<Command> goToReef(Reef reef) {
        return ()-> DriveCommands.pathfindPose(() -> reef.getclosestBranch(AutoBuilder.getCurrentPose(), Level.L3).getPose());
    }
}
