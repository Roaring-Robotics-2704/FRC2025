// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.components;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.source.SourceChooser;
import frc.robot.commands.drive.DriveCommands;
import java.util.function.Supplier;

/** Add your docs here. */
public class GoToSource {
    public static Supplier<Command> goToSource(SourceChooser chooser) {
        return () -> DriveCommands.pathfindPose(chooser::getSourcePose);
    }
}
