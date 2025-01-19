package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private MutDistance setpoint = Inches.mutable(0);

    public Elevator(ElevatorIO io) {
        this.io = io;
        io.init();
    }

    @Override
    public void periodic() {
        this.io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
        this.io.runSetpoint(setpoint);
    }

    public Command setSetpoint(Distance setpoint) {
        return runOnce(() -> this.setpoint.mut_replace(setpoint));
    }
}
