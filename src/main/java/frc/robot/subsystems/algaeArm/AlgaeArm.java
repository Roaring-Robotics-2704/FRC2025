// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algaeArm;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class AlgaeArm extends SubsystemBase {
    private AlgaeArmIO algaeArmIO;
    private final AlgaeArmIOInputsAutoLogged inputs = new AlgaeArmIOInputsAutoLogged();
    Rotation2d setpoint = Rotation2d.kZero;

    public AlgaeArm(AlgaeArmIO algaeArmIO) {
        this.algaeArmIO = algaeArmIO;
    }

    /** Creates a new algaeArm. */
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        algaeArmIO.updateInputs(inputs);
        algaeArmIO.setAlgaeArmPosition(setpoint);
        Logger.recordOutput("Arm/Setpoint", setpoint.getDegrees());
    }

    public void setPivotAngle(Double angle) {
        setpoint = Rotation2d.fromRadians(angle);
    }

    public void runVolts(Voltage volts) {
        algaeArmIO.setAlgaeArmVoltage(volts.in(Volts));
    }

    public void runRollers(double speed) {
        algaeArmIO.setRollerSpeed(speed);
    }
}
