package frc.robot.commands.Autonomus.reef;

import frc.robot.Locations.ReefChooser;
import frc.robot.Locations.ReefLocations;
import frc.robot.commands.Autonomus.reef.Branch.Level;
import frc.robot.commands.Autonomus.reef.Branch.Side;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

public class Reef {
    Face[] faces = new Face[6];

    // Private constructor to hide the implicit public one
    public Reef() {

        faces[0] = new Face(F_LEFT, F_RIGHT);
        faces[1] = new Face(FL_LEFT, FL_RIGHT);
        faces[2] = new Face(BL_LEFT, BL_RIGHT);
        faces[3] = new Face(B_LEFT, B_RIGHT);
        faces[4] = new Face(BR_LEFT, BR_RIGHT);
        faces[5] = new Face(FR_LEFT, FR_RIGHT);
    }

    class Face {
        Branch rightBranch;
        Branch leftBranch;
        Boolean isSelected = true;

        public Face(Pose2d leftPose, Pose2d rightPose) {
            rightBranch = new Branch(Side.RIGHT, rightPose);
            leftBranch = new Branch(Side.LEFT, leftPose);
        }

        public Branch getBranch(Side side) {
            if (side == Side.RIGHT) {
                return rightBranch;
            } else {
                return leftBranch;
            }
        }

        public void setSelected(Boolean status) {
            isSelected = status;
        }

        public Boolean getSelected() {
            return isSelected;
        }
    }

    public Branch[] checkHeightAvailability(Level level) {
        List<Branch> branches = new ArrayList<>();
        for (Face face : faces) {
            if (face.getSelected()) {
                Branch rightBranch = face.getBranch(Side.RIGHT);
                Branch leftBranch = face.getBranch(Side.LEFT);
                if (rightBranch.getCoralStatus(level)) {
                    branches.add(rightBranch);
                }
                if (leftBranch.getCoralStatus(level)) {
                    branches.add(leftBranch);
                }
            }
        }
        return branches.toArray(new Branch[branches.size()]);
    }

    public Branch getClosestBranch(Pose2d currentPose, Branch[] branches) {
        Branch closestBranch = null;
        double minDistance = Double.MAX_VALUE;
        for (Branch branch : branches) {
            double distance =
                    currentPose.getTranslation().getDistance(branch.getPose().getTranslation());
            if (distance < minDistance) {
                minDistance = distance;
                closestBranch = branch;
            }
        }
        return closestBranch;
    }

    @AutoLogOutput
    public Pose2d getclosestPose(Pose2d currentPose, Level level) {
        Level currentLevel = level;
        Branch[] branches = checkHeightAvailability(currentLevel);
        while (branches.length == 0) {
            currentLevel = getLesserLevel(level);
            branches = checkHeightAvailability(currentLevel);
        }
        System.out.println(getClosestBranch(currentPose, checkHeightAvailability(level))
                .getPose()
                .toString());
        return getClosestBranch(currentPose, checkHeightAvailability(level)).getPose();
    }

    // Front Left Reef locations
    public static final Pose2d FL_RIGHT = new Pose2d(3.703, 5.06, Rotation2d.fromDegrees(-60));
    public static final Pose2d FL_LEFT = new Pose2d(3.987, 5.224, Rotation2d.fromDegrees(-60));

    // Front Reef locations
    public static final Pose2d F_LEFT = new Pose2d(3.2, 4.19, Rotation2d.fromDegrees(0));
    public static final Pose2d F_RIGHT = new Pose2d(3.2, 3.862, Rotation2d.fromDegrees(0));

    // Front Right Reef locations
    public static final Pose2d FR_LEFT = new Pose2d(3.703, 2.992, Rotation2d.fromDegrees(60));
    public static final Pose2d FR_RIGHT = new Pose2d(3.987, 2.828, Rotation2d.fromDegrees(60));

    // Back Left Reef locations
    public static final Pose2d BL_LEFT = new Pose2d(5.276, 5.06, Rotation2d.fromDegrees(-120));
    public static final Pose2d BL_RIGHT = new Pose2d(4.992, 5.224, Rotation2d.fromDegrees(-120));

    // Back Reef locations
    public static final Pose2d B_LEFT = new Pose2d(5.778, 3.862, Rotation2d.fromDegrees(180));
    public static final Pose2d B_RIGHT = new Pose2d(5.778, 4.19, Rotation2d.fromDegrees(180));

    // Back Right Reef locations
    public static final Pose2d BR_LEFT = new Pose2d(4.992, 2.828, Rotation2d.fromDegrees(120));
    public static final Pose2d BR_RIGHT = new Pose2d(5.276, 2.992, Rotation2d.fromDegrees(120));

    // private static CoralStatus[] getHeightPriority(CoralStatus priority1) {
    //     // CoralStatus[] priorities = {CoralStatus.L3, CoralStatus.L2, CoralStatus.L1,
    //     // CoralStatus.L4};
    //     CoralStatus priority2 = getLesserPriority(priority1);
    //     CoralStatus priority3 = getLesserPriority(priority2);
    //     CoralStatus priority4 = getLesserPriority(priority3);
    //     return new CoralStatus[] { priority1, priority2, priority3, priority4 };
    // }

    private static Level getLesserLevel(Level priority) {
        if (priority == Level.L4) {
            return Level.L3;
        } else if (priority == Level.L3) {
            return Level.L2;
        } else if (priority == Level.L2) {
            return Level.L4;
        } else if (priority == Level.L1) {
            return Level.L4;
        } else {
            return Level.L3;
        }
    }
    
}
