// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.buttonBoard;

import static frc.robot.subsystems.buttonBoard.ButtonBoardConstants.*;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.auto.reef.Branch;
import frc.robot.auto.reef.Branch.Level;
import frc.robot.auto.reef.Branch.Side;
import frc.robot.auto.reef.Reef;
import frc.robot.auto.reef.Reef.Faces;

public class ButtonBoard extends SubsystemBase {
    Reef reef;
    Level currentLevel = Level.L3;
    GenericHID board = new GenericHID(ButtonBoardConstants.BB_PORT);
    Branch.Side currentSide = Side.RIGHT;
    Boolean[] selectedFaces = new Boolean[5];
    /** Creates a new ButtonBoard. */
    public ButtonBoard(Reef reef) {
        this.reef = reef;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (board.getRawButtonPressed(REEF_NEAR_CENTER)) {
            reef.getReefSide(Faces.FRONT)
                    .setSelected(!reef.getReefSide(Faces.FRONT).getSelected());
        }
        if (board.getRawButtonPressed(REEF_NEAR_LEFT)) {
            reef.getReefSide(Faces.FRONT_LEFT)
                    .setSelected(!reef.getReefSide(Faces.FRONT_LEFT).getSelected());
        }
        if (board.getRawButtonPressed(REEF_NEAR_RIGHT)) {
            reef.getReefSide(Faces.FRONT_RIGHT)
                    .setSelected(!reef.getReefSide(Faces.FRONT_RIGHT).getSelected());
        }
        if (board.getRawButtonPressed(REEF_FAR_LEFT)) {
            reef.getReefSide(Faces.BACK_LEFT)
                    .setSelected(!reef.getReefSide(Faces.BACK_LEFT).getSelected());
        }
        if (board.getRawButtonPressed(REEF_FAR_RIGHT)) {
            reef.getReefSide(Faces.BACK_RIGHT)
                    .setSelected(!reef.getReefSide(Faces.BACK_RIGHT).getSelected());
        }
        if (board.getRawButtonPressed(REEF_FAR_CENTER)) {
            reef.getReefSide(Faces.BACK)
                    .setSelected(!reef.getReefSide(Faces.BACK).getSelected());
        }

        if (board.getRawButton(ButtonBoardConstants.LEVEL_4R)) {
            currentLevel = Level.L4;
            currentSide = Side.RIGHT;
        } else if (board.getRawButton(ButtonBoardConstants.LEVEL_4L)) {
            currentLevel = Level.L4;
            currentSide = Side.LEFT;
        } else if (board.getRawButton(ButtonBoardConstants.LEVEL_3R)) {
            currentLevel = Level.L3;
            currentSide = Side.RIGHT;
        } else if (board.getRawButton(ButtonBoardConstants.LEVEL_3L)) {
            currentLevel = Level.L3;
            currentSide = Side.LEFT;
        } else if (board.getRawButton(ButtonBoardConstants.LEVEL_2R)) {
            currentLevel = Level.L2;
            currentSide = Side.RIGHT;
        } else if (board.getRawButton(ButtonBoardConstants.LEVEL_2L)) {
            currentLevel = Level.L2;
            currentSide = Side.LEFT;
        } else if (board.getRawButton(ButtonBoardConstants.LEVEL_1R)) {
            currentLevel = Level.L1;
            currentSide = Side.RIGHT;
        } else if (board.getRawButton(ButtonBoardConstants.LEVEL_1L)) {
            currentLevel = Level.L1;
            currentSide = Side.LEFT;
        }
    }

    public Side getSelectedBranchSide() {
        return currentSide;
    }

    public Level getSelectedLevel() {
        return currentLevel;
    }
}
