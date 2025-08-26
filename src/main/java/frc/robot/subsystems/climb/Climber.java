// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SparkUtil;

public class Climber extends SubsystemBase {
    /** Creates a new Climber. */
    SparkMax climberMotor;

    public Climber() {
        climberMotor = new SparkMax(ClimbConstants.CLIMBER_MOTOR_ID, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();

        config.smartCurrentLimit(30).voltageCompensation(12);

        SparkUtil.tryUntilOk(
                climberMotor,
                5,
                () -> climberMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void setClimberSpeed(double speed) {
        // Set the speed of the climber motor
        climberMotor.set(speed);
    }

    public Command climbUpCommand() {
        return new RunCommand(() -> setClimberSpeed(0.8), this).finallyDo(() -> setClimberSpeed(0));
    }

    public Command climbDownCommand() {
        return new RunCommand(() -> setClimberSpeed(-0.8), this).finallyDo(() -> setClimberSpeed(0));
    }

    public Command climbStop() {
        return new RunCommand(() -> setClimberSpeed(0), this).finallyDo(() -> setClimberSpeed(0));
    }
}
