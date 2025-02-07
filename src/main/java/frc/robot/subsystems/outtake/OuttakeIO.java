package frc.robot.subsystems.outtake;

import org.littletonrobotics.junction.AutoLog;

// Sets up outtake variables
public interface OuttakeIO {
    @AutoLog
    class OuttakeIOInputs {
        public boolean connected = false;
        public double velocityRadPerSec = 0.0;
        public double voltage = 0.0;
        public double currentAmps = 0.0;
        public boolean outtakeLoaded = false;
    }

    default void setSpeed(double speed) {}

    default void updateInputs(OuttakeIOInputs inputs) {}
}
