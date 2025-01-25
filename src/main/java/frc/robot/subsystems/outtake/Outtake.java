// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.outtake;
import static frc.robot.subsystems.outtake.OuttakeConstants.INTAKE_SPEED;
import static frc.robot.subsystems.outtake.OuttakeConstants.OUTTAKE_SPEED;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Outtake extends SubsystemBase {
    private OuttakeIO outtake;

    private final OuttakeIOInputsAutoLogged outtakeInputs = new OuttakeIOInputsAutoLogged();

    /** Creates a new Outtake. */
    public Outtake(OuttakeIO outtake) {
        this.outtake = outtake;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    Command outtakeCommand(){

        return new RunCommand(()-> outtake.setSpeed(OUTTAKE_SPEED));
    }

    Command intakeCommand(){

        return new RunCommand(()-> outtake.setSpeed(INTAKE_SPEED));
    }

}
