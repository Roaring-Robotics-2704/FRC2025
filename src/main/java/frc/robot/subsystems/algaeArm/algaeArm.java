// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algaeArm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorIOInputsAutoLogged;

public class AlgaeArm extends SubsystemBase {
    private AlgaeArmIO algaeArmIO;
    private final AlgaeArmIOInputsAutoLogged inputs = new AlgaeArmIOInputsAutoLogged();

    

    public AlgaeArm(AlgaeArmIO algaeArmIO) {
        this.algaeArmIO = algaeArmIO;
    }

    /** Creates a new algaeArm. */
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        algaeArmIO.updateInputs(inputs);
    }
    public void setPivotAngle(Double angle) {
        algaeArmIO.setAlgaeArmPosition(angle);
    }
}
