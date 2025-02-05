// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.algaeArm.AlgaeArm;
import frc.robot.subsystems.algaeArm.AlgaeArmConstants;

/** Add your docs here. */
public class AlgaeArmFactory {

	public static Command AlgaeArmIntake(AlgaeArm arm) {
		return new RunCommand(()->arm.setPivotAngle(AlgaeArmConstants.INTAKE_POSITION), arm);
	}
	public static Command AlgaeArmHold(AlgaeArm arm) {
		return new RunCommand(()->arm.setPivotAngle(AlgaeArmConstants.HOLD_POSITION), arm);
	}

	public static Command AlgaeArmInside(AlgaeArm arm) {
		return new RunCommand(()->arm.setPivotAngle(AlgaeArmConstants.INSIDE_POSITION), arm);
	}
}
