package frc.robot.subsystems.remover;

import org.littletonrobotics.junction.AutoLog;

public interface RemoverIO {
    @AutoLog
    class RemoverIOInputs {
        public double RemoverRollerVelocity = 0.0;
        public double RemoverRollerAmps = 0.0;
        public double RemoverRollerVoltage = 0.0;
        public double RemoverRollerSpeed = 0.0;
        public double RemoverPivotPositionRad = 0.0;
        public double RemoverPivotAppliedVolts = 0.0;
        public double RemoverPivotVelocity = 0.0;
        public double RemoverPivotAmps = 0.0;
    }

    default void setRemoverRollerSpeed(double speed) {}

    default void updateInputs(RemoverIOInputs inputs) {}

    default void setRemoverRollerVoltage(double voltage) {}

    default void setRemoverRollerPositionRad(double positionRad) {}
}
