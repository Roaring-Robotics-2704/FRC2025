// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.reef;

import edu.wpi.first.math.geometry.Pose2d;

/** Add your docs here. */
public class Branch {
    private Boolean L4 = false;
    private Boolean L3 = false;
    private Boolean L2 = false;
    private Boolean L1 = false;
    private Pose2d pose;
    Side side = null;

    public Branch(Side side, Pose2d pose) {
        this.side = side;
        this.pose = pose;
    }

    public enum Side {
        LEFT,
        RIGHT
    }

    public enum Level {
        L4,
        L3,
        L2,
        L1
    }

    public Side getSide() {
        return side;
    }

    public void setCoralStatus(Level level, Boolean status) {
        switch (level) {
            case L4:
                L4 = status;
                break;
            case L3:
                L3 = status;
                break;
            case L2:
                L2 = status;
                break;
            case L1:
                L1 = status;
                break;
        }
    }

    public Boolean getCoralStatus(Level level) {
        switch (level) {
            case L4:
                return L4;
            case L3:
                return L3;
            case L2:
                return L2;
            case L1:
                return false;
            default:
                return false;
        }
    }

    public Pose2d getPose() {
        return pose;
    }
}
