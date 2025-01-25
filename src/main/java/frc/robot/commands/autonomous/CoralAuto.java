// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.Auto.utilities.Pathbuilder;

/** Add your docs here. */
public class CoralAuto {
    private Command reefOuttake = Pathbuilder.goToReef();
    private Command sourceIntake = Pathbuilder.goToSource();

    public Command getCommand() {
        return new RepeatCommand(reefOuttake.andThen(sourceIntake)).repeatedly().withTimeout(15);
    }
}
