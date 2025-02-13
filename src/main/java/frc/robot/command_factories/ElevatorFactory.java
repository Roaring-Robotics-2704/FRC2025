// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;

/** Add your docs here. */
public class ElevatorFactory {
    private ElevatorFactory() {}

    public static Command elevatorL1(Elevator elevator) {
        return new RunCommand(() -> elevator.setElevatorHeight(ElevatorConstants.L1_HEIGHT), elevator);
    }

    public static Command elevatorL2(Elevator elevator) {
        return new RunCommand(() -> elevator.setElevatorHeight(ElevatorConstants.L2_HEIGHT), elevator);
    }

    public static Command elevatorL3(Elevator elevator) {
        return new RunCommand(() -> elevator.setElevatorHeight(ElevatorConstants.L3_HEIGHT), elevator);
    }

    public static Command elevatorL4(Elevator elevator) {
        return new RunCommand(() -> elevator.setElevatorHeight(ElevatorConstants.L4_HEIGHT), elevator);
    }
}
