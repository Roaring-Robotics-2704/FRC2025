// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algaeArm;


public interface algaeArmIO {
  class algaeArmIOInputs{

    public boolean algaeArmConnected = false;
    public double algaeArmPositionRad = 0.0;
    public double algaeArmVelocity = 0.0;
    public double algaeArmAppliedVolts = 0.0;
    public double algaeArmCurrentAmps = 0.0;
  }
  
  default void updateInputs(algaeArmIOInputs inputs) {}

  default void setAlgaeArmVoltage(double voltage) {}

  default void setAlgaeArmPosition(double positionRad) {}

  default void setAlgaeArmVelocity(double velocityRadPerSec) {}

}