// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import static frc.robot.Constants.PRIORITY_LEVEL;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.auto.reef.Branch.Level;
import frc.robot.auto.reef.Reef;
import frc.robot.command_factories.ElevatorFactory;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.outtake.Outtake;
import java.util.function.Supplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DynamicAutoBeta extends SequentialCommandGroup {
    private final Reef reef;
    private final Elevator elevator;
    private final Outtake outtake;
    private boolean first = true;

    /** Creates a new DynamicAutoBeta. */
    public DynamicAutoBeta(Reef reef, Drive drive, Elevator elevator, Outtake outtake) {
        this.reef = reef;
        this.elevator = elevator;
        this.outtake = outtake;
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addRequirements(drive, elevator, outtake);
        addCommands(
                new PrintCommand("Dynamic Auto Beta starting Cycle"), // Added because WPILIB skips the first command
                Commands.deferredProxy(RobotContainer.GoToReef()),
                Commands.deferredProxy(ElevatorUp()),
                Commands.deferredProxy(Outtake()),
                Commands.deferredProxy(FillReefSlot()),
                Commands.deferredProxy(ElevatorDown()),
                Commands.deferredProxy(RobotContainer.GoToSource()),
                Commands.deferredProxy(Intake()));
    }

    public Supplier<Command> ElevatorUp() {
        return () -> {
            Level currentLevel = PRIORITY_LEVEL;
            if (!reef.getclosestBranch(AutoBuilder.getCurrentPose(), PRIORITY_LEVEL)
                    .getCoralStatus(PRIORITY_LEVEL)) {
                return Commands.runOnce(() -> ElevatorFactory.elevator(elevator, PRIORITY_LEVEL));
            }
            while (reef.getclosestBranch(AutoBuilder.getCurrentPose(), PRIORITY_LEVEL)
                    .getCoralStatus(currentLevel)) {
                currentLevel = Reef.getLesserLevel(currentLevel);
            }
            Level freeLevel = currentLevel;
            return Commands.runOnce(() -> ElevatorFactory.elevator(elevator, freeLevel));
        };
    }

    public Supplier<Command> ElevatorDown() {
        return () -> Commands.runOnce(() -> ElevatorFactory.elevatorIntake(elevator));
    }

    public Supplier<Command> Outtake() {
        return () -> outtake.outtakeOutCmd();
    }

    public Supplier<Command> Intake() {
        return () -> outtake.outtakeInCmd();
    }

    public Supplier<Command> FillReefSlot() {
        return () -> {
            Level currentLevel = PRIORITY_LEVEL;
            if (!reef.getclosestBranch(AutoBuilder.getCurrentPose(), currentLevel)
                    .getCoralStatus(PRIORITY_LEVEL)) {
                reef.getclosestBranch(AutoBuilder.getCurrentPose(), PRIORITY_LEVEL)
                        .setCoralStatus(PRIORITY_LEVEL, true);
            }
            while (reef.getclosestBranch(AutoBuilder.getCurrentPose(), currentLevel)
                    .getCoralStatus(currentLevel)) {
                currentLevel = Reef.getLesserLevel(currentLevel);
            }
            reef.getclosestBranch(AutoBuilder.getCurrentPose(), currentLevel).setCoralStatus(currentLevel, true);
            return new PrintCommand("Filled reef slot");
        };
    }
}
