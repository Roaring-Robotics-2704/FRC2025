// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algaeArm;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeArmIO {
    @AutoLog
    class AlgaeArmIOInputs {

        public double algaePivotPositionRad = 0.0;
        public double algaePivotVelocity = 0.0;
        public double algaePivotAppliedVolts = 0.0;
        public double algaePivotAmps = 0.0;
        public double algaeRollerVelocity = 0.0;
        public double algaeRollerAmps = 0.0;
        public double algaeRollerVoltage = 0.0;
        public double algaeRollerSpeed = 0.0;
    }

    default void updateInputs(AlgaeArmIOInputs inputs) {}

    default void setAlgaeArmVoltage(double voltage) {}

    default void setAlgaeArmPosition(double positionRad) {}

    default void setRollerSpeed(double speed) {}
}
