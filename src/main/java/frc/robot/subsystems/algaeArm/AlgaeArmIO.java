// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algaeArm;

import static frc.robot.subsystems.algaeArm.AlgaeArmConstants.INSIDE_POSITION;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface AlgaeArmIO {
    /**
     * Represents the input data for the Algae Arm subsystem. This class contains various sensor readings and state
     * information related to the Algae Arm's pivot and roller mechanisms.
     */
    @AutoLog
    class AlgaeArmIOInputs {

        public double algaePivotPosition = INSIDE_POSITION;
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

    default void setAlgaeArmPosition(Rotation2d rotation) {}

    default void setRollerSpeed(double speed) {}
}
