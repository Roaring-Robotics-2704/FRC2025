// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AnalogPotentiometer;

/** Add your docs here. */
public class ElevatorIOSpark implements ElevatorIO {

    private final SparkMax leftElevatorMotor;
    private final SparkMax rightElevatorMotor;
    private PIDController pidController;
    private final AnalogPotentiometer elevatorEncoder;
    private final ElevatorFeedforward feedforward;

    public ElevatorIOSpark() {
        leftElevatorMotor = new SparkMax(ElevatorConstants.ELEVATOR_MOTOR_1, MotorType.kBrushless);
        rightElevatorMotor = new SparkMax(ElevatorConstants.ELEVATOR_MOTOR_2, MotorType.kBrushless);

        elevatorEncoder = new AnalogPotentiometer(ElevatorConstants.ANALOG_INPUT);
        pidController = new PIDController(
                ElevatorConstants.ELEVATOR_KP, ElevatorConstants.ELEVATOR_KI, ElevatorConstants.ELEVATOR_KD);
        pidController.setIntegratorRange(-12, 12);
        feedforward = new ElevatorFeedforward(kS, kG, kV, kA);

        // Configure drive motor
        var driveConfig = new SparkMaxConfig();
        driveConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(CURRENT_LIMIT).voltageCompensation(12.0);
        driveConfig.signals.appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);
        tryUntilOk(
                leftElevatorMotor,
                5,
                () -> leftElevatorMotor.configure(
                        driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        tryUntilOk(
                rightElevatorMotor,
                5,
                () -> rightElevatorMotor.configure(
                        driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.elevatorConnected = (!leftElevatorMotor.getFaults().can && !rightElevatorMotor.getFaults().can);
        inputs.elevatorHeight = getHeight().in(Meters);
        inputs.leftElevatorCurrentAmps = leftElevatorMotor.getOutputCurrent();
        inputs.rightElevatorCurrentAmps = rightElevatorMotor.getOutputCurrent();
        inputs.leftElevatorCurrentAmps = leftElevatorMotor.getOutputCurrent();
        inputs.rightElevatorCurrentAmps = rightElevatorMotor.getOutputCurrent();
    }

    @Override
    public void runVolts(Voltage volts) {
        leftElevatorMotor.setVoltage(volts);
        rightElevatorMotor.setVoltage(volts);
    }

    @Override
    public void runSetpoint(Distance setpoint) {
        double output = MathUtil.clamp(pidController.calculate(getHeight().in(Meters), setpoint.in(null)), -12, 12);
        leftElevatorMotor.set(output);
        rightElevatorMotor.set(output);
    }

    private Distance getHeight() {
        return Meters.of(elevatorEncoder.get() * 120);
    }
}
