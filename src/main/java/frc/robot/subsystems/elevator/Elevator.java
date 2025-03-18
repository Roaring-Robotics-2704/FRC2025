// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.MAX_ELEVATOR_VOLTAGE;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

    boolean manual = false;
    /** Creates a new Elevator. */
    private final ElevatorIO io;

    private final RobotContainer robotContainer;

    private final ElevatorVisualization visualization = new ElevatorVisualization();

    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private ProfiledPIDController controller = new ProfiledPIDController(
            ElevatorConstants.ELEVATOR_KP,
            ElevatorConstants.ELEVATOR_KI,
            ElevatorConstants.ELEVATOR_KD,
            new TrapezoidProfile.Constraints(3, 1));

    private ElevatorFeedforward feedforward = new ElevatorFeedforward(
            ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);

    private SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volts.of(0.5).per(Second),
                    Volts.of(2),
                    null,
                    state -> Logger.recordOutput("Elevator/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(this::setElevatorVolts, null, local()));

    public Elevator(ElevatorIO io, RobotContainer robotContainer) {
        SmartDashboard.putData("Elevator PID", controller);
        this.io = io;
        this.robotContainer = robotContainer;
        io.init();
    }

    @Override
    public void periodic() {
        this.io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
        Logger.recordOutput("Elevator/Goal", controller.getGoal().position);
        Logger.recordOutput("Elevator/Height", inputs.elevatorHeight);
        Logger.recordOutput("Elevator/Setpoint", controller.getSetpoint().position);
        visualization.update(inputs.elevatorHeight);
        io.runVolts(Volts.of(MathUtil.clamp(
                controller.calculate(inputs.elevatorHeight) + feedforward.calculate(controller.getSetpoint().velocity),
                -MAX_ELEVATOR_VOLTAGE,
                MAX_ELEVATOR_VOLTAGE)));

        // This method will be called once per scheduler run
    }

    public void setElevatorHeight(double height) {
        if ((height > inputs.elevatorHeight) && !robotContainer.hasCoral()) {
        } else {
            controller.setGoal(height);
        }
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

    private Elevator local() {
        return this;
    }

    public SysIdRoutine getSysIdRoutine() {
        return sysIdRoutine;
    }

    public double getHeight() {
        return inputs.elevatorHeight;
    }
}
