// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
    /** Creates a new Elevator. */
    private final ElevatorIO io;

    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private MutDistance setpoint = Inches.mutable(0);

    public Elevator(ElevatorIO io) {
        this.io = io;
        io.init();
    }

    @Override
    public void periodic() {
        this.io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
        this.io.runSetpoint(setpoint);
        // This method will be called once per scheduler run
    }

    public void setElevatorHeight(double height) {
        io.setElevatorPosition(height);
    }
}
