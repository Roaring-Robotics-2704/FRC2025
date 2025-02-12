// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.reef.Reef;
import frc.robot.auto.source.SourceChooser;
import frc.robot.commands.autonomous.components.GoToReef;
import frc.robot.commands.autonomous.components.GoToSource;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DynamicAutoV2 extends Command {
  Reef reef;
  SourceChooser chooser;
  Command reefCommand;
  Command sourceCommand;
  boolean reefDone = false;
    /** Creates a new DynamicAutoV2. */
  public DynamicAutoV2(Reef reef, SourceChooser chooser) {
    this.reef = reef;
    this.chooser = chooser;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    reefCommand = GoToReef.goToReef(reef).get();
    sourceCommand = GoToSource.goToSource(chooser).get();
    reefCommand.initialize();
    reefDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    reefCommand = GoToReef.goToReef(reef).get();
    sourceCommand = GoToSource.goToSource(chooser).get();
    if(reefCommand.isFinished() && !reefDone) {
      reefDone = true;
      reefCommand.end(false);
      sourceCommand.initialize();
      sourceCommand.execute();
    } else if (sourceCommand.isFinished() && reefDone) {
      reefDone = false;
      reefCommand.initialize();
      reefCommand.execute();
    }
    if (reefDone) {
      sourceCommand.execute();
    } else {
      reefCommand.execute();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    reefCommand.end(interrupted);
    sourceCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
