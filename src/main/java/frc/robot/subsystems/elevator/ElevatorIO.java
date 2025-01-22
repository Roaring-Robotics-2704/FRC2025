package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    class ElevatorIOInputs {
        public boolean elevatorConnected = false;
        public double elevatorHeight = 0.0;
        public double elevatorVelocity = 0.0;
        public double elevatorAppliedVolts = 0.0;
        public double elevatorCurrentAmps = 0.0;
    }

    /** Updates the set of loggable inputs. */
    default void updateInputs(ElevatorIOInputs inputs) {}

    default void setElevatorHeight(double outputs) {}

    default void setElevatorVelocity(double velocityRadperSec) {}
  }

