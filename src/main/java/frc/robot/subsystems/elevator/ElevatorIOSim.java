// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

/** Add your docs here. */
public class ElevatorIOSim implements ElevatorIO {
    PIDController pidController = new PIDController(
            ElevatorConstants.ELEVATOR_KP, ElevatorConstants.ELEVATOR_KI, ElevatorConstants.ELEVATOR_KD);

    private final ElevatorSim m_elevatorSim = new ElevatorSim(
            ElevatorConstants.ELEVATOR_GEARBOX,
            ElevatorConstants.GEAR_REDUCTION,
            ElevatorConstants.CARRIAGE_MASS,
            ElevatorConstants.PULLEY_RADIUS,
            0,
            Units.inchesToMeters(72),
            true,
            0,
            0.01,
            0.0);

    public ElevatorIOSim() {}

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {

        m_elevatorSim.update(0.02);
        inputs.elevatorHeight = m_elevatorSim.getPositionMeters();
        inputs.elevatorVelocity = m_elevatorSim.getVelocityMetersPerSecond();
        inputs.leftElevatorCurrentAmps = m_elevatorSim.getCurrentDrawAmps();
        inputs.elevatorConnected = true;
    }

    @Override
    public void runVolts(Voltage volts) {
        m_elevatorSim.setInputVoltage(volts.in(Volts));
    }

    @Override
    public void runSetpoint(Distance setpoint) {
        double output = MathUtil.clamp(
                pidController.calculate(m_elevatorSim.getPositionMeters(), setpoint.in(Meters)), -12, 12);
        m_elevatorSim.setInputVoltage(output);
    }
}
