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

    /** Creates a new DynamicAutoBeta. */
    public DynamicAutoBeta(Reef reef, Drive drive, Elevator elevator, Outtake outtake) {
        this.reef = reef;
        this.elevator = elevator;
        this.outtake = outtake;
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addRequirements(drive, elevator, outtake); // Add subsystem dependencies
        addCommands(
                new PrintCommand("Dynamic Auto Beta starting Cycle"), // Print command to indicate start
                new PrintCommand("Dynamic Auto Beta going to Reef"),
                Commands.defer(RobotContainer.GoToReef(true, true), getRequirements())
                        .andThen(new PrintCommand("goToReef Done")), // Go to reef
                new PrintCommand("Dynamic Auto Beta raising Elevator"),
                ElevatorUp().get(), // Move elevator up
                new PrintCommand("Dynamic Auto Beta outtaking"),
                Outtake().get(), // Outtake command
                new PrintCommand("Dynamic Auto Beta filling reef slot"),
                Commands.defer(FillReefSlot(), getRequirements()), // Fill reef slot
                Commands.parallel(
                        ElevatorDown().get(), // Move elevator down
                        Commands.defer(RobotContainer.GoToSource(), getRequirements()) // Go to source
                        ),
                new PrintCommand("Dynamic Auto Beta intaking"),
                Intake().get()); // Intake command
    }

    public Supplier<Command> ElevatorUp() {
        return () -> {
            Level currentLevel = PRIORITY_LEVEL; // Set current level to priority level
            if (!reef.getclosestBranch(AutoBuilder.getCurrentPose(), PRIORITY_LEVEL, true)
                    .getCoralStatus(PRIORITY_LEVEL)) { // Check if coral status is false
                return Commands.runOnce(
                        () -> ElevatorFactory.elevator(elevator, PRIORITY_LEVEL)); // Run elevator command
            }
            while (reef.getclosestBranch(AutoBuilder.getCurrentPose(), PRIORITY_LEVEL, true)
                    .getCoralStatus(currentLevel)) { // Loop to find free level
                currentLevel = Reef.getLesserLevel(currentLevel); // Get lesser level
            }
            Level freeLevel = currentLevel; // Set free level
            return Commands.runOnce(() -> ElevatorFactory.elevator(elevator, freeLevel)); // Run elevator command
        };
    }

    public Supplier<Command> ElevatorDown() {
        return () -> Commands.runOnce(() -> ElevatorFactory.elevatorIntake(elevator)); // Run elevator intake command
    }

    public Supplier<Command> Outtake() {
        return () -> outtake.outtakeOutCmd(); // Run outtake command
    }

    public Supplier<Command> Intake() {
        return () -> outtake.outtakeInCmd(); // Run intake command
    }

    public Supplier<Command> FillReefSlot() {
        return () -> Commands.runOnce(() -> {
            Level currentLevel = PRIORITY_LEVEL; // Set current level to priority level
            if (!reef.getclosestBranch(AutoBuilder.getCurrentPose(), currentLevel, true)
                    .getCoralStatus(PRIORITY_LEVEL)) { // Check if coral status is false
                reef.getclosestBranch(AutoBuilder.getCurrentPose(), PRIORITY_LEVEL, true)
                        .setCoralStatus(PRIORITY_LEVEL, true); // Set coral status to true
            } else {
                while (reef.getclosestBranch(AutoBuilder.getCurrentPose(), currentLevel, true)
                        .getCoralStatus(currentLevel)) { // Loop to find free level
                    currentLevel = Reef.getLesserLevel(currentLevel); // Get lesser level
                }
                reef.getclosestBranch(AutoBuilder.getCurrentPose(), currentLevel, true)
                        .setCoralStatus(currentLevel, true); // Set coral status to true
            }
        });
    }
}
