// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.buttonBoard;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.auto.reef.Reef;
import frc.robot.auto.reef.Reef.Faces;

public class ButtonBoard extends SubsystemBase {
  Reef reef;
  GenericHID board = new GenericHID(ButtonBoardConstants.BUTTON_BOARD_PORT);
  /** Creates a new ButtonBoard. */
  public ButtonBoard(Reef reef) {
    this.reef = reef;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    reef.getReefSide(Faces.FRONT).setSelected(true);
  }
}
