// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algaeArm;
import frc.robot.subsystems.algaeArm.algaeArm;
import frc.robot.subsystems.algaeArm.algaeArmConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class algaeArm extends SubsystemBase {
  private algaeArmIOSpark algaeArm;


    /** Creates a new Outtake. */
    public algaeArm(algaeArmIOSpark algaeArm) {
        this.algaeArm = algaeArm;
    }
    

  /** Creates a new algaeArm. */
  public algaeArm() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  Command pivotCommand() {
    return new RunCommand(()-> algaeArm.setPivotSpeed(algaeArmConstants.PIVOT_SPEED));
  }

  Command rollerCommand() {
    return new RunCommand(()-> algaeArm.setRollerSpeed(algaeArmConstants.ROLLERS_SPEED));
  }
}
