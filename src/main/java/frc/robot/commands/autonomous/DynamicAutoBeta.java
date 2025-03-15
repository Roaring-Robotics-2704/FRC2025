// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import frc.robot.auto.reef.Reef;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.outtake.Outtake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DynamicAutoBeta {
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
        //     addCommands(
        //             new PrintCommand("Dynamic Auto Beta starting Cycle"), // Print command to indicate start
        //             new PrintCommand("Dynamic Auto Beta going to Reef"),
        //             Commands.defer(RobotContainer.GoToReef(true, false), getRequirements())
        //                     .andThen(new PrintCommand("goToReef Done")), // Go to reef
        //             new PrintCommand("Dynamic Auto Beta raising Elevator"),
        //             ElevatorUp().get(), // Move elevator up
        //             new PrintCommand("Dynamic Auto Beta outtaking"),
        //             Outtake().get(), // Outtake command
        //             new PrintCommand("Dynamic Auto Beta filling reef slot"),
        //             Commands.defer(FillReefSlot(), getRequirements()), // Fill reef slot
        //             Commands.parallel(
        //                     ElevatorDown().get(), // Move elevator down
        //                     Commands.defer(RobotContainer.GoToSource(), getRequirements()) // Go to source
        //                     ),
        //             new PrintCommand("Dynamic Auto Beta intaking"),
        //             Intake().get()); // Intake command
    }
}
