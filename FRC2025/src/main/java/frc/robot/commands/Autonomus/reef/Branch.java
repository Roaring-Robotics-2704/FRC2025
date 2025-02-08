package frc.robot.commands.Autonomus.reef;

import edu.wpi.first.math.geometry.Pose2d;

/** Add your docs here. */
public class Branch {
    private Boolean l4 = false;
    private Boolean l3 = false;
    private Boolean l2 = false;
    private Boolean l1 = false;
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
                l4 = status;
                break;
            case L3:
                l3 = status;
                break;
            case L2:
                l2 = status;
                break;
            case L1:
                l1 = status;
                break;
        }
    }

    public Boolean getCoralStatus(Level level) {
        switch (level) {
            case L4:
                return l4;
            case L3:
                return l3;
            case L2:
                return l2;
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
