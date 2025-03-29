// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algaeArm;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.algaeArm.AlgaeArmConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import org.littletonrobotics.junction.Logger;

public class AlgaeArm extends SubsystemBase {
    private double goal;
    private AlgaeArmIO algaeArmIO;
    private final AlgaeArmIOInputsAutoLogged inputs = new AlgaeArmIOInputsAutoLogged();
    private SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volts.of(0.5).per(Second),
                    Volts.of(1),
                    null,
                    state -> Logger.recordOutput("Arm/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(this::runVolts, null, this));

    private ProfiledPIDController controller =
            new ProfiledPIDController(ALGAE_ARM_KP, ALGAE_ARM_KI, ALGAE_ARM_KD, new Constraints(180, 45));

    private ArmFeedforward feedforward = new ArmFeedforward(KS, KG, KV);

    public AlgaeArm(AlgaeArmIO algaeArmIO) {
        this.algaeArmIO = algaeArmIO;
        goal = inputs.algaePivotPosition;
    }

    /** Creates a new algaeArm. */
    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        algaeArmIO.updateInputs(inputs);
        if (controller.getGoal().position != goal) controller.setGoal(goal);
        double requestedVoltage = MathUtil.clamp(
                controller.calculate(inputs.algaePivotPosition)
                        + feedforward.calculate(controller.getSetpoint().position, controller.getSetpoint().velocity),
                -10,
                10);
        Logger.recordOutput("Arm/Goal", controller.getGoal().position);
        Logger.recordOutput("Arm/Setpoint", controller.getSetpoint().position);
        Logger.recordOutput("Arm/RequestedVoltage", requestedVoltage);
        algaeArmIO.setAlgaeArmVoltage(requestedVoltage);
        Logger.recordOutput("Arm/Measured", inputs.algaePivotPosition);
        Logger.recordOutput("Arm/Voltage", inputs.algaePivotAppliedVolts);
        Logger.recordOutput("Arm/Velocity", inputs.algaePivotVelocity);
        Logger.recordOutput("Arm/Current", inputs.algaePivotAmps);
    }

    public void setPivotAngle(Double angle) {
        goal = angle;
        controller.setGoal(angle);
    }

    public void runVolts(Voltage volts) {
        algaeArmIO.setAlgaeArmVoltage(volts.in(Volts));
    }

    public void runRollers(double speed) {
        algaeArmIO.setRollerSpeed(speed);
    }

    public Command sysIdDynamic(Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    public Command sysIdStatic(Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }
}
