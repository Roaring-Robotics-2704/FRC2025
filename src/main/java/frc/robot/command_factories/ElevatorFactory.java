// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.auto.reef.Branch.Level;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;

/**
 * Factory class for creating elevator commands.
 */
public class ElevatorFactory {
    private ElevatorFactory() {} // Private constructor to prevent instantiation

    /**
     * Creates a command to move the elevator to the specified level.
     *
     * @param elevator the Elevator instance
     * @param level the target level
     * @return the command to move the elevator to the specified level
     */
    public static Command elevator(Elevator elevator, Level level) {
        switch (level) {
            case L1:
                return elevatorL1(elevator); // Move to level 1
            case L2:
                return elevatorL2(elevator); // Move to level 2
            case L3:
                return elevatorL3(elevator); // Move to level 3
            case L4:
                return elevatorL4(elevator); // Move to level 4
            default:
                return elevatorL3(elevator); // Default to level 3
        }
    }

    /**
     * Creates a command to move the elevator to level 1.
     *
     * @param elevator the Elevator instance
     * @return the command to move the elevator to level 1
     */
    public static Command elevatorL1(Elevator elevator) {
        return new RunCommand(() -> elevator.setElevatorHeight(ElevatorConstants.L1_HEIGHT), elevator); // Set height to level 1
    }

    /**
     * Creates a command to move the elevator to level 2.
     *
     * @param elevator the Elevator instance
     * @return the command to move the elevator to level 2
     */
    public static Command elevatorL2(Elevator elevator) {
        return new RunCommand(() -> elevator.setElevatorHeight(ElevatorConstants.L2_HEIGHT), elevator); // Set height to level 2
    }

    /**
     * Creates a command to move the elevator to level 3.
     *
     * @param elevator the Elevator instance
     * @return the command to move the elevator to level 3
     */
    public static Command elevatorL3(Elevator elevator) {
        return new RunCommand(() -> elevator.setElevatorHeight(ElevatorConstants.L3_HEIGHT), elevator); // Set height to level 3
    }

    /**
     * Creates a command to move the elevator to level 4.
     *
     * @param elevator the Elevator instance
     * @return the command to move the elevator to level 4
     */
    public static Command elevatorL4(Elevator elevator) {
        return new RunCommand(() -> elevator.setElevatorHeight(ElevatorConstants.L4_HEIGHT), elevator); // Set height to level 4
    }

    /**
     * Creates a command to move the elevator to the intake position.
     *
     * @param elevator the Elevator instance
     * @return the command to move the elevator to the intake position
     */
    public static Command elevatorIntake(Elevator elevator) {
        return new RunCommand(() -> elevator.setElevatorHeight(ElevatorConstants.INTAKE_HEIGHT), elevator); // Set height to intake position
    }

    /**
     * Creates a command to manually move the elevator up.
     *
     * @param elevator the Elevator instance
     * @return the command to manually move the elevator up
     */
    public static Command manualElevatorUp(Elevator elevator) {
        return new RunCommand(() -> elevator.setElevatorVolts(3), elevator) // Apply upward voltage
                .repeatedly()
                .finallyDo(() -> elevator.setElevatorVolts(0)); // Stop voltage when command ends
    }

    /**
     * Creates a command to manually move the elevator down.
     *
     * @param elevator the Elevator instance
     * @return the command to manually move the elevator down
     */
    public static Command manualElevatorDown(Elevator elevator) {
        return new RunCommand(() -> elevator.setElevatorVolts(-3), elevator) // Apply downward voltage
                .repeatedly()
                .finallyDo(() -> elevator.setElevatorVolts(0)); // Stop voltage when command ends
    }
}
