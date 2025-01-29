// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.Inches;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.AnalogEncoder;

/** Add your docs here. */
public class ElevatorIOSpark implements ElevatorIO{

	private final SparkMax leftElevatorMotor;
    private final SparkMax rightElevatorMotor;
    private  PIDController pidController;
    private final AnalogEncoder elevatorEncoder;

	public ElevatorIOSpark(){
		leftElevatorMotor = new SparkMax(ElevatorConstants.ELEVATOR_MOTOR_1, MotorType.kBrushless);
		rightElevatorMotor = new SparkMax(ElevatorConstants.ELEVATOR_MOTOR_2, MotorType.kBrushless);
		elevatorEncoder = new AnalogEncoder(ElevatorConstants.ANALOG_INPUT);
		pidController = new PIDController(ElevatorConstants.ELEVATOR_KP, ElevatorConstants.ELEVATOR_KI, ElevatorConstants.ELEVATOR_KD);
		pidController.enableContinuousInput(-Math.PI, Math.PI);
	}

	@Override
	public void setElevatorHeight(double height) {
		leftElevatorMotor.set(pidController.calculate(elevatorEncoder.get(), height));
		rightElevatorMotor.set(pidController.calculate(elevatorEncoder.get(), height));
		//figure out if .get() returns radians or vertical distance
	}

	@Override
		public void updateInputs(ElevatorIOInputs inputs) {
			inputs.elevatorConnected = (!leftElevatorMotor.getFaults().can && !rightElevatorMotor.getFaults().can);
			inputs.elevatorHeight = leftElevatorMotor.getEncoder().getPosition();
			inputs.elevatorVelocity = leftElevatorMotor.getEncoder().getVelocity();
			inputs.leftElevatorCurrentAmps = leftElevatorMotor.getOutputCurrent();
			inputs.rightElevatorCurrentAmps = rightElevatorMotor.getOutputCurrent();
			inputs.leftElevatorCurrentAmps = leftElevatorMotor.getOutputCurrent();
			inputs.rightElevatorCurrentAmps = rightElevatorMotor.getOutputCurrent();
		}
}


