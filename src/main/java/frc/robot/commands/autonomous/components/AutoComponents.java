// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.components;

import static frc.robot.subsystems.drive.DriveConstants.CONSTRAINTS;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.reef.Branch.Level;
import frc.robot.auto.reef.Reef;
import java.util.function.Supplier;

/** Add your docs here. */
public class AutoComponents {
    public static Command goToReef(Reef reef) {
        return AutoBuilder.pathfindToPose(reef.getclosestPose(AutoBuilder.getCurrentPose(), Level.L3), CONSTRAINTS);
    }

    public static Command goToSource(Supplier<Pose2d> sourceChooser) {
        return AutoBuilder.pathfindToPose(sourceChooser.get(), CONSTRAINTS);
    }
}
