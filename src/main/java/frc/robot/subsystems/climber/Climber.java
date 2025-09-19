// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
    /** Creates a new Climber. */
    SparkMax climberMotor = new SparkMax(22, SparkMax.MotorType.kBrushless);

    public Climber() {
        var driveConfig = new SparkMaxConfig();
        driveConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12.0);
        driveConfig.signals.appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);
        tryUntilOk(
                climberMotor,
                5,
                () -> climberMotor.configure(
                        driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        Logger.recordOutput("Climber/AppliedPercent", climberMotor.get());
        Logger.recordOutput("Climber/AppliedCurrent", climberMotor.getOutputCurrent());
        Logger.recordOutput("Climber/Voltage", climberMotor.getBusVoltage());
        Logger.recordOutput("Climber/DutyCycle", climberMotor.getAppliedOutput());
    }

    public void setClimberMotor(double speed) {
        climberMotor.set(speed);
    }

    public void stopClimberMotor() {
        climberMotor.set(0);
    }

    public Command climb(double speed) {
        return Commands.run(() -> setClimberMotor(speed), this)
                .withName("Climb")
                .finallyDo(() -> stopClimberMotor());
    }
}
