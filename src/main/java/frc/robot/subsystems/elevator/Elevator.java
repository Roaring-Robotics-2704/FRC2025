// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  public Elevator() {}
    private final CANSparkMax FRONT_ELEVATOR;
    private final CANSparkMax RIGHT_ELEVATOR;

    private final AbsoluteEncoder FRONT_ELEVATOR_ENCODER;
    private final AbsoluteEncoder LEFT_ELEVATOR_ENCODER;

    double 


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


}
