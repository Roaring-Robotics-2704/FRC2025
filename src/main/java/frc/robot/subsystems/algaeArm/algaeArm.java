// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algaeArm;

import frc.robot.subsystems.algaeArm.AlgaeArmConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeArm extends SubsystemBase {
  private AlgaeArmIO algaeArmIO;



    public AlgaeArm(AlgaeArmIO algaeArmIO) {
        this.algaeArmIO = algaeArmIO;
    }
    

  /** Creates a new algaeArm. */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  Command pivotCommand() {
    return new RunCommand(()-> algaeArmIO.setAlgaeArmPosition(AlgaeArmConstants.PIVOT_SPEED));
  }

  Command rollerCommand() {
    return new RunCommand(()-> algaeArmIO.setRollerSpeed(AlgaeArmConstants.ROLLERS_SPEED));
  }
}
