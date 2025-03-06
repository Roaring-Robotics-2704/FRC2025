// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

    boolean manual = false;
    /** Creates a new Elevator. */
    private final ElevatorIO io;

    private final ElevatorVisualization visualization = new ElevatorVisualization();

    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final TrapezoidProfile m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(2, 1));

    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();

    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    public Elevator(ElevatorIO io) {
        this.io = io;
        io.init();
    }

    @Override
    public void periodic() {
        this.io.updateInputs(inputs);
        setpoint = m_profile.calculate(0.02, setpoint, m_goal);
        Logger.processInputs("Elevator", inputs);
        Logger.recordOutput("Elevator/Goal", m_goal.position);
        Logger.recordOutput("Elevator/Height", inputs.elevatorHeight);
        Logger.recordOutput("Elevator/Setpoint", setpoint.position);

        this.io.runSetpoint(setpoint);
        visualization.update(inputs.elevatorHeight);

        // This method will be called once per scheduler run
    }

    public void setElevatorHeight(double height) {
        m_goal = new State(height, 0);
    }

    public void setElevatorVolts(double volts) {
        io.runVolts(Volts.of(volts));
    }

    public void setElevatorVolts(Voltage volts) {
        io.runVolts(volts);
    }

    public double getVelocity() {
        return inputs.elevatorVelocity;
    }
}
