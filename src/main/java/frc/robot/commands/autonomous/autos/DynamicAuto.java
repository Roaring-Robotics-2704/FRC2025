// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.autonomous.components.GoToReef;
import frc.robot.commands.autonomous.components.GoToSource;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.AutoLogOutput;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DynamicAuto extends Command {
    Command m_selectedCommand;
    Command goToReef;
    Command goToSource;
    Boolean atReef = false;

    /** Creates a new DynamicAuto. */
    public DynamicAuto(SendableChooser<Pose2d> chooser, Drive drive) {
        goToReef = new GoToReef(drive);
        goToSource = new GoToSource(chooser, drive);
        addRequirements(drive);
    }

    @AutoLogOutput
    private Command getNextCommand(boolean toSource) {
        if (toSource) {
            return goToSource;
        } else {
            return goToReef;
        }
    }

    @Override
    public void initialize() {
        m_selectedCommand = getNextCommand(atReef);
        m_selectedCommand.initialize();
    }

    @Override
    public void execute() {

        // Command currentCommand = m_selectedCommand;
        // Command nextCommand = getNextCommand(atReef);
        // if (!currentCommand.isFinished()) {
        //     currentCommand.execute();
        // } else {
        //     currentCommand.cancel();
        //     atReef = !atReef;
        //     nextCommand = getNextCommand(atReef);
        //     nextCommand.initialize();
        //     nextCommand.execute();
        // }
        // m_selectedCommand = nextCommand;
    }

    @Override
    public void end(boolean interrupted) {
        m_selectedCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return m_selectedCommand.isFinished();
    }
}
