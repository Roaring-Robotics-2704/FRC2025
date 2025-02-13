// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.outtake;

import static frc.robot.subsystems.outtake.OuttakeConstants.INTAKE_SPEED;
import static frc.robot.subsystems.outtake.OuttakeConstants.INTAKE_TIME;
import static frc.robot.subsystems.outtake.OuttakeConstants.OUTTAKE_SPEED;
import static frc.robot.subsystems.outtake.OuttakeConstants.OUTTAKE_TIME;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Outtake extends SubsystemBase {
    private OuttakeIO outtake; // Creates a new outtake

    private final OuttakeIOInputsAutoLogged outtakeInputs = new OuttakeIOInputsAutoLogged();

    // Defines the new outtake
    public Outtake(OuttakeIO outtake) {
        this.outtake = outtake;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    Command outtakeOutCmd() { // Runs outtake motor with set times and speeds (Go to OuttakeConstants.java to change)

        return new RunCommand(() -> outtake.setSpeed(OUTTAKE_SPEED))
                .repeatedly()
                .withTimeout(OUTTAKE_TIME);
    }

    Command outtakeInCmd() {

        return new RunCommand(() -> outtake.setSpeed(INTAKE_SPEED)).repeatedly().withTimeout(INTAKE_TIME);
    }

    Command outtakeOutSlowCmd() { // Runs outtake motor with set times and speeds (Go to OuttakeConstants.java to
        // change)

        return new RunCommand(() -> outtake.setSpeed(OUTTAKE_SPEED * .5))
                .repeatedly()
                .withTimeout(OUTTAKE_TIME * 2);
    }
}
