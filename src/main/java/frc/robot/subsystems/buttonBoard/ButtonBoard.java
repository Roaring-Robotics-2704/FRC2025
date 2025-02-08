// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.buttonBoard;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.auto.reef.Reef;
import frc.robot.auto.reef.Branch.Level;
import frc.robot.auto.reef.Reef.Faces;
import frc.robot.auto.reef.Branch.Side;


public class ButtonBoard extends SubsystemBase {
  Reef reef;
  Level currentLevel = Level.L3;
  GenericHID board = new GenericHID(ButtonBoardConstants.BB_PORT);
  /** Creates a new ButtonBoard. */
  public ButtonBoard(Reef reef) {
    this.reef = reef;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    reef.getReefSide(Faces.FRONT).setSelected(board.getRawButton(ButtonBoardConstants.REEF_NEAR_CENTER));
    reef.getReefSide(Faces.FRONT_LEFT).setSelected(board.getRawButton(ButtonBoardConstants.REEF_NEAR_LEFT));
    reef.getReefSide(Faces.FRONT_RIGHT).setSelected(board.getRawButton(ButtonBoardConstants.REEF_NEAR_RIGHT));
    reef.getReefSide(Faces.BACK_LEFT).setSelected(board.getRawButton(ButtonBoardConstants.REEF_FAR_LEFT));
    reef.getReefSide(Faces.BACK_RIGHT).setSelected(board.getRawButton(ButtonBoardConstants.REEF_FAR_RIGHT));
    reef.getReefSide(Faces.BACK).setSelected(board.getRawButton(ButtonBoardConstants.REEF_FAR_CENTER));

    if(board.getRawButton(ButtonBoardConstants.LEVEL_4R)) {
      currentLevel = Level.L4;
    }
    else if(board.getRawButton(ButtonBoardConstants.LEVEL_4L)) {
      currentLevel = Level.L4;
    }
    else if(board.getRawButton(ButtonBoardConstants.LEVEL_3R)) {
      currentLevel = Level.L3;
    }
    else if(board.getRawButton(ButtonBoardConstants.LEVEL_3L)) {
      currentLevel = Level.L3;
    }
    else if(board.getRawButton(ButtonBoardConstants.LEVEL_2R)) {
      currentLevel = Level.L2;
    }
    else if(board.getRawButton(ButtonBoardConstants.LEVEL_2L)) {
      currentLevel = Level.L2;
    }
    else if(board.getRawButton(ButtonBoardConstants.LEVEL_1R)) {
      currentLevel = Level.L1;
    }
    else if(board.getRawButton(ButtonBoardConstants.LEVEL_1L)) {
      currentLevel = Level.L1;
    }
  }

  public Side getSelectedBranchSide() {
    if(ButtonBoardConstants.LEVEL_1L==1 || ButtonBoardConstants.LEVEL_2L==1 || ButtonBoardConstants.LEVEL_3L==1 || ButtonBoardConstants.LEVEL_4L==1) {
      return Side.LEFT;
    }
    else {
      return Side.RIGHT;
    }

  }


  public Level getSelectedLevel() {
    return currentLevel;
  }
  
}
