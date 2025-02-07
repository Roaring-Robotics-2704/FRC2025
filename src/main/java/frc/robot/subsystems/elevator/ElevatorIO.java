package frc.robot.subsystems.elevator;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    class ElevatorIOInputs {
        public boolean elevatorConnected = false;
        public double elevatorHeight = 0.0;
        public double elevatorVelocity = 0.0;
        public double rightElevatorAppliedVolts = 0.0;
        public double leftElevatorAppliedVolts = 0.0;
        public double rightElevatorCurrentAmps = 0.0;
        public double leftElevatorCurrentAmps = 0.0;
    }

    /** Updates the set of loggable inputs. */
    default void updateInputs(ElevatorIOInputs inputs) {}

    default void setElevatorPosition(double outputs) {}

    default void setElevatorVelocity(double velocityRadperSec) {}

    default void runVolts(Voltage volts) {};

    default void runSetpoint(Distance position) {};

    default void stop() {};

    default void init() {};
}
