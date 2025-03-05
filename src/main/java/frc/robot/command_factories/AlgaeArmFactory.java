// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command_factories;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.algaeArm.AlgaeArm;
import frc.robot.subsystems.algaeArm.AlgaeArmConstants;

/** Factory class for creating commands to control the AlgaeArm. */
public class AlgaeArmFactory {

    /**
     * Creates a command to move the AlgaeArm to the intake position.
     *
     * @param arm the AlgaeArm instance
     * @return the command to move the arm to the intake position
     */
    public static Command AlgaeArmIntake(AlgaeArm arm) {
        return new RunCommand(() -> arm.setPivotAngle(AlgaeArmConstants.INTAKE_POSITION), arm);
    }

    /**
     * Creates a command to hold the AlgaeArm in the hold position.
     *
     * @param arm the AlgaeArm instance
     * @return the command to hold the arm in the hold position
     */
    public static Command AlgaeArmHold(AlgaeArm arm) {
        return new RunCommand(() -> arm.setPivotAngle(AlgaeArmConstants.HOLD_POSITION), arm);
    }

    /**
     * Creates a command to move the AlgaeArm to the inside position.
     *
     * @param arm the AlgaeArm instance
     * @return the command to move the arm to the inside position
     */
    public static Command AlgaeArmInside(AlgaeArm arm) {
        return new RunCommand(() -> arm.setPivotAngle(AlgaeArmConstants.INSIDE_POSITION), arm);
    }

    /**
     * Creates a command to manually move the AlgaeArm up.
     *
     * @param arm the AlgaeArm instance
     * @return the command to manually move the arm up
     */
    public static Command manualAlgaeArmUp(AlgaeArm arm) {
        return new RunCommand(() -> arm.runVolts(Volts.of(4)), arm)
                .repeatedly()
                .finallyDo(() -> arm.runVolts(Volts.zero()));
    }

    /**
     * Creates a command to manually move the AlgaeArm down.
     *
     * @param arm the AlgaeArm instance
     * @return the command to manually move the arm down
     */
    public static Command manualAlgaeArmDown(AlgaeArm arm) {
        return new RunCommand(() -> arm.runVolts(Volts.of(-4)), arm)
                .repeatedly()
                .finallyDo(() -> arm.runVolts(Volts.zero()));
    }

    /**
     * Creates a command to manually run the AlgaeArm rollers inward.
     *
     * @param arm the AlgaeArm instance
     * @return the command to manually run the rollers inward
     */
    public static Command manualAlgaeRollerIn(AlgaeArm arm) {
        return new RunCommand(() -> arm.runRollers(0.5), arm).repeatedly().finallyDo(() -> arm.runRollers(0));
    }

    /**
     * Creates a command to manually run the AlgaeArm rollers outward.
     *
     * @param arm the AlgaeArm instance
     * @return the command to manually run the rollers outward
     */
    public static Command manualAlgaeRollerOut(AlgaeArm arm) {
        return new RunCommand(() -> arm.runRollers(-0.5), arm).repeatedly().finallyDo(() -> arm.runRollers(0));
    }
}
