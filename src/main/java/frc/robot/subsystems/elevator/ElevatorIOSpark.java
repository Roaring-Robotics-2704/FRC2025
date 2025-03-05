// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.elevator.ElevatorConstants.CURRENT_LIMIT;
import static frc.robot.subsystems.elevator.ElevatorConstants.kA;
import static frc.robot.subsystems.elevator.ElevatorConstants.kG;
import static frc.robot.subsystems.elevator.ElevatorConstants.kS;
import static frc.robot.subsystems.elevator.ElevatorConstants.kV;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AnalogPotentiometer;

/** Add your docs here. */
public class ElevatorIOSpark implements ElevatorIO {

    private final SparkMax leftElevatorMotor;
    private final SparkMax rightElevatorMotor;
    private PIDController pidController;
    private final AnalogPotentiometer elevatorEncoder = new AnalogPotentiometer(ElevatorConstants.ANALOG_INPUT);
    private double offset = 0;
    private static final ElevatorFeedforward feedForward = new ElevatorFeedforward(kS, kG, kV, kA);
    private double previousHeight = 0.0;

    public ElevatorIOSpark() {
        leftElevatorMotor = new SparkMax(ElevatorConstants.ELEVATOR_MOTOR_1, MotorType.kBrushless);
        rightElevatorMotor = new SparkMax(ElevatorConstants.ELEVATOR_MOTOR_2, MotorType.kBrushless);
        pidController = new PIDController(
                ElevatorConstants.ELEVATOR_KP, ElevatorConstants.ELEVATOR_KI, ElevatorConstants.ELEVATOR_KD);
        pidController.setIntegratorRange(-12, 12);

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
        offset = getHeight().in(Meters);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.elevatorConnected = (!leftElevatorMotor.getFaults().can && !rightElevatorMotor.getFaults().can);
        inputs.elevatorHeight = getHeight().in(Meters);
        inputs.leftElevatorCurrentAmps = leftElevatorMotor.getOutputCurrent();
        inputs.rightElevatorCurrentAmps = rightElevatorMotor.getOutputCurrent();
        inputs.leftElevatorCurrentAmps = leftElevatorMotor.getOutputCurrent();
        inputs.rightElevatorCurrentAmps = rightElevatorMotor.getOutputCurrent();
        inputs.elevatorVelocity = getVelocity();
        inputs.leftElevatorAppliedVolts = leftElevatorMotor.getAppliedOutput() * 12;
        inputs.rightElevatorAppliedVolts = rightElevatorMotor.getAppliedOutput() * 12;
        inputs.elevatorSetpoint = pidController.getSetpoint();
    }

    @Override
    public void runVolts(Voltage volts) {
        leftElevatorMotor.setVoltage(volts);
        rightElevatorMotor.setVoltage(volts);
    }

    @Override
    public void runSetpoint(TrapezoidProfile.State setpoint) {
        double output = MathUtil.clamp(
                pidController.calculate(getHeight().in(Meters), setpoint.position)
                        + feedForward.calculate(setpoint.velocity),
                -3,
                3);
        leftElevatorMotor.set(output);
        rightElevatorMotor.set(output);
    }

    private Distance getHeight() {

        return Meters.of((elevatorEncoder.get() * Units.inchesToMeters(120)) - offset);
    }

    private double getVelocity() {
        // Calculate the velocity by comparing the current height with the previous height
        double currentHeight = getHeight().in(Meters);
        double prevHeight = previousHeight;
        previousHeight = currentHeight;

        double deltaTime = 0.02; // Assuming this method is called every 20ms
        return (currentHeight - prevHeight) / deltaTime;
    }
}
