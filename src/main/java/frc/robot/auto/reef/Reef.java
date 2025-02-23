// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.reef;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;
import frc.robot.auto.reef.Branch.Level;
import frc.robot.auto.reef.Branch.Side;
import frc.robot.util.PoseUtil;
import java.util.ArrayList;
import java.util.List;

/** Add your docs here. */
public class Reef {
    static Face[] faces = new Face[6];
    static int twelve = 3;

    // Private constructor to hide the implicit public one
    public Reef() {

        faces[0] = new Face(F_LEFT, F_RIGHT, Faces.FRONT);
        faces[1] = new Face(FL_LEFT, FL_RIGHT, Faces.FRONT_LEFT);
        faces[2] = new Face(BL_LEFT, BL_RIGHT, Faces.BACK_LEFT);
        faces[3] = new Face(B_LEFT, B_RIGHT, Faces.BACK);
        faces[4] = new Face(BR_LEFT, BR_RIGHT, Faces.BACK_RIGHT);
        faces[5] = new Face(FR_LEFT, FR_RIGHT, Faces.FRONT_RIGHT);
        // if (Constants.CURRENT_MODE != Mode.SIM && !Constants.COMPETITION) {
        //     faces[1].setSelected(false);
        //     faces[2].setSelected(false);
        //     faces[3].setSelected(false);
        //     faces[4].setSelected(false);
        //     faces[5].setSelected(false);
        // }
        // faces[3].setSelected(false);
    }

    public class Face {
        Branch rightBranch;
        Branch leftBranch;
        Boolean isSelected = true;
        Faces face;

        public Face(Pose2d leftPose, Pose2d rightPose, Faces face) {
            rightBranch = new Branch(Side.RIGHT, rightPose);
            leftBranch = new Branch(Side.LEFT, leftPose);
            this.face = face;
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

        public String getName() {
            return face.name();
        }
    }

    public enum Faces {
        FRONT(faces[0]),
        FRONT_LEFT(faces[1]),
        FRONT_RIGHT(faces[2]),
        BACK_LEFT(faces[3]),
        BACK_RIGHT(faces[4]),
        BACK(faces[5]);

        private final Face face;

        Faces(Face face) {
            this.face = face;
        }

        public Face getFace() {
            return face;
        }
    }

    public Branch[] checkHeightAvailability(Level level) {
        List<Branch> branches = new ArrayList<>();
        for (Face face : faces) {
            if (face.getSelected()) {
                Branch rightBranch = face.getBranch(Side.RIGHT);
                Branch leftBranch = face.getBranch(Side.LEFT);
                if (!rightBranch.getCoralStatus(level)) {
                    branches.add(rightBranch);
                }
                if (!leftBranch.getCoralStatus(level)) {
                    branches.add(leftBranch);
                }
            }
        }
        return branches.toArray(new Branch[branches.size()]);
    }

    public Branch getclosestBranch(Pose2d currentPose, Level level) {
        Pose2d pose = currentPose;
        System.out.println("X: " + pose.getX() + " Y: " + pose.getY());
        if (Robot.isRedAlliance()) {
            pose = FlippingUtil.flipFieldPose(currentPose);
        }
        Level currentLevel = level;
        Branch[] branches = checkHeightAvailability(currentLevel);
        while (branches.length == 0) {
            currentLevel = getLesserLevel(currentLevel);
            branches = checkHeightAvailability(currentLevel); // TODO add recursion
        }
        Branch closestBranch = null;
        double minDistance = Double.MAX_VALUE;
        for (Branch branch : branches) {
            double distance = PoseUtil.getDistance(pose, branch.getPose());
            //    0 System.out.println(distance);
            // currentPose.getTranslation().getDistance(branch.getPose().getTranslation());
            if (distance < minDistance) {
                minDistance = distance;
                closestBranch = branch;
            }
        }

        return closestBranch;
    }

    public Face getclosestFace(Pose2d currentPose) {
        if (Robot.isRedAlliance()) {
            currentPose = FlippingUtil.flipFieldPose(currentPose);
        }
        List<Face> availablefaces = new ArrayList<>();
        for (Face face : faces) {
            if (face.getSelected()) {
                availablefaces.add(face);
            }
        }
        if (availablefaces.isEmpty()) {
            availablefaces.add(Faces.FRONT.getFace());
        }
        Face closestFace = null;
        double minDistance = Double.MAX_VALUE;
        for (Face face : availablefaces) {
            double distance = currentPose
                    .getTranslation()
                    .getDistance(PoseUtil.averagePose(face.leftBranch.getPose(), face.rightBranch.getPose())
                            .getTranslation());
            if (distance < minDistance) {
                minDistance = distance;
                closestFace = face;
            }
        }

        return closestFace;
    }

    public boolean isReefFull() {
        for (Face face : faces) {
            if (face.getSelected() && (!face.leftBranch.isFull() || !face.rightBranch.isFull())) {
                return false;
            }
        }
        return true;
    }

    // Front Left Reef locations
    private static final Pose2d FL_RIGHT = new Pose2d(3.703, 5.06, Rotation2d.fromDegrees(-60));
    private static final Pose2d FL_LEFT = new Pose2d(3.987, 5.224, Rotation2d.fromDegrees(-60));

    // Front Reef locations
    private static final Pose2d F_LEFT = new Pose2d(3.2, 4.19, Rotation2d.fromDegrees(0));
    private static final Pose2d F_RIGHT = new Pose2d(3.2, 3.862, Rotation2d.fromDegrees(0));

    // Front Right Reef locations
    private static final Pose2d FR_LEFT = new Pose2d(3.703, 2.992, Rotation2d.fromDegrees(60));
    private static final Pose2d FR_RIGHT = new Pose2d(3.987, 2.828, Rotation2d.fromDegrees(60));

    // Back Left Reef locations
    private static final Pose2d BL_LEFT = new Pose2d(5.276, 5.06, Rotation2d.fromDegrees(-120));
    private static final Pose2d BL_RIGHT = new Pose2d(4.992, 5.224, Rotation2d.fromDegrees(-120));

    // Back Reef locations
    private static final Pose2d B_LEFT = new Pose2d(5.778, 3.862, Rotation2d.fromDegrees(180));
    private static final Pose2d B_RIGHT = new Pose2d(5.778, 4.19, Rotation2d.fromDegrees(180));

    // Back Right Reef locations
    private static final Pose2d BR_LEFT = new Pose2d(4.992, 2.828, Rotation2d.fromDegrees(120));
    private static final Pose2d BR_RIGHT = new Pose2d(5.276, 2.992, Rotation2d.fromDegrees(120));

    // private static CoralStatus[] getHeightPriority(CoralStatus priority1) {
    // // CoralStatus[] priorities = {CoralStatus.L3, CoralStatus.L2,
    // CoralStatus.L1,
    // // CoralStatus.L4};
    // CoralStatus priority2 = getLesserPriority(priority1);
    // CoralStatus priority3 = getLesserPriority(priority2);
    // CoralStatus priority4 = getLesserPriority(priority3);
    // return new CoralStatus[] { priority1, priority2, priority3, priority4 };
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

    public Face getReefSide(Faces face) {
        if (face == Faces.FRONT) {
            return faces[0];
        } else if (face == Faces.FRONT_LEFT) {
            return faces[1];
        } else if (face == Faces.BACK_LEFT) {
            return faces[2];
        } else if (face == Faces.BACK) {
            return faces[3];
        } else if (face == Faces.BACK_RIGHT) {
            return faces[4];
        } else if (face == Faces.FRONT_RIGHT) {
            return faces[5];
        } else return faces[0];
    }
}
