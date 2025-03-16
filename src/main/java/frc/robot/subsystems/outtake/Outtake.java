// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.outtake;

import static frc.robot.subsystems.outtake.OuttakeConstants.INTAKE_SPEED;
import static frc.robot.subsystems.outtake.OuttakeConstants.OUTTAKE_SPEED;
import static frc.robot.subsystems.outtake.OuttakeConstants.REVERSE_SPEED;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.Elevator;

public class Outtake extends SubsystemBase {
    private OuttakeIO outtake; // Creates a new outtake
    private Elevator elevator;
    private boolean canReverse = true;

    private final OuttakeIOInputsAutoLogged outtakeInputs = new OuttakeIOInputsAutoLogged();

    // Defines the new outtake
    public Outtake(OuttakeIO outtake, Elevator elevator) {
        this.outtake = outtake;
        this.elevator = elevator;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        outtake.updateInputs(outtakeInputs);
        canReverse = (elevator.getHeight() <= OuttakeConstants.NO_REVERSE_HEIGHT);
    }

    public Command outtakeOutCmd(
            boolean useSensor) { // Runs outtake motor with set times and speeds (Go to OuttakeConstants.java to change)
        return Commands.either(
                new RunCommand(() -> outtake.setSpeed(OUTTAKE_SPEED))
                        .repeatedly()
                        .until(() -> !outtakeInputs.outtakeLoaded)
                        .finallyDo(() -> outtake.setSpeed(0)),
                new RunCommand(() -> outtake.setSpeed(OUTTAKE_SPEED))
                        .repeatedly()
                        .finallyDo(() -> outtake.setSpeed(0)),
                () -> useSensor);
    }

    public Command outtakeInCmd(boolean useSensor) {

        return Commands.either(
                new RunCommand(() -> outtake.setSpeed(INTAKE_SPEED))
                        .repeatedly()
                        .until(() -> outtakeInputs.outtakeLoaded)
                        .finallyDo(() -> outtake.setSpeed(0)),
                new RunCommand(() -> outtake.setSpeed(INTAKE_SPEED))
                        .repeatedly()
                        .finallyDo(() -> outtake.setSpeed(0)),
                () -> useSensor);
    }

    public Command outtakeReverseCMD() { // Runs outtake motor with set times and speeds (Go to OuttakeConstants.java to
        // change)

        return Commands.either(
                new RunCommand(() -> outtake.setSpeed(REVERSE_SPEED))
                        .repeatedly()
                        .finallyDo(() -> outtake.setSpeed(0)),
                new Command() {},
                () -> canReverse);
    }

    public boolean isLoaded() {
        return outtakeInputs.outtakeLoaded;
    }

    // public Command manualOuttakeCMD() {
    //     return new RunCommand(() -> outtake.setSpeed(OUTTAKE_SPEED))
    //             .repeatedly()
    //             .finallyDo(() -> outtake.setSpeed(0));
    // }

    // public Command manualIntakeCMD() {
    //     return new RunCommand(() -> outtake.setSpeed(INTAKE_SPEED)).repeatedly().finallyDo(() ->
    // outtake.setSpeed(0));
    // }
}
