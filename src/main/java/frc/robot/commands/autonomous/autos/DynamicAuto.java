// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.autos;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.reef.Reef;
import frc.robot.subsystems.drive.Drive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DynamicAuto extends Command {
    private Reef reef;
    private Drive drive;
    EventLoop loop = new EventLoop();
    Trigger toReef;
    Trigger toSource;

    // if we're at the target velocity, kick the ball into the shooter wheel
    /** Creates a new DynamicAuto. */
    public DynamicAuto(Reef reef, Drive drive) {
        this.reef = reef;
        this.drive = drive;
        addRequirements(drive);
        toReef = new Trigger(loop, () -> !toSource.getAsBoolean());
        toSource = new Trigger(loop, () -> !toReef.getAsBoolean());

        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
