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

/**
 * The Reef class represents a collection of faces, each containing two branches. It provides methods to check the
 * availability of branches at different levels, find the closest branch or face to a given pose, and determine if the
 * reef is full.
 */
public class Reef {
    // Array to hold the six faces of the reef
    static Face[] faces = new Face[6];

    // Private constructor to initialize the faces with their respective positions and orientations
    public Reef() {
        faces[0] = new Face(F_LEFT, F_RIGHT, FaceEnum.FRONT);
        faces[1] = new Face(FL_LEFT, FL_RIGHT, FaceEnum.FRONT_LEFT);
        faces[2] = new Face(BL_LEFT, BL_RIGHT, FaceEnum.BACK_LEFT);
        faces[3] = new Face(B_LEFT, B_RIGHT, FaceEnum.BACK);
        faces[4] = new Face(BR_LEFT, BR_RIGHT, FaceEnum.BACK_RIGHT);
        faces[5] = new Face(FR_LEFT, FR_RIGHT, FaceEnum.FRONT_RIGHT);
    }

    /** The Face class represents a face of the reef, containing two branches and a selection status. */
    public class Face {
        Branch rightBranch;
        Branch leftBranch;
        Boolean isSelected = true;
        FaceEnum face;

        /**
         * Constructor to initialize the face with left and right branch poses and its type.
         *
         * @param leftPose Pose of the left branch
         * @param rightPose Pose of the right branch
         * @param face Type of the face
         */
        public Face(Pose2d leftPose, Pose2d rightPose, FaceEnum face) {
            rightBranch = new Branch(Side.RIGHT, rightPose, this);
            leftBranch = new Branch(Side.LEFT, leftPose, this);
            this.face = face;
        }

        /**
         * Get the branch on the specified side.
         *
         * @param side Side of the branch (LEFT or RIGHT)
         * @return The branch on the specified side
         */
        public Branch getBranch(Side side) {
            if (side == Side.RIGHT) {
                return rightBranch;
            } else {
                return leftBranch;
            }
        }

        /**
         * Set the selection status of the face.
         *
         * @param status Selection status (true if selected, false otherwise)
         */
        public void setSelected(Boolean status) {
            isSelected = status;
        }

        /**
         * Get the selection status of the face.
         *
         * @return Selection status (true if selected, false otherwise)
         */
        public Boolean getSelected() {
            return isSelected;
        }

        /**
         * Get the name of the face.
         *
         * @return Name of the face
         */
        public String getName() {
            return face.name();
        }
    }

    /** Enum representing the different faces of the reef. */
    public enum FaceEnum {
        FRONT(faces[0]),
        FRONT_LEFT(faces[1]),
        FRONT_RIGHT(faces[2]),
        BACK_LEFT(faces[3]),
        BACK_RIGHT(faces[4]),
        BACK(faces[5]);

        private final Face face;

        FaceEnum(Face face) {
            this.face = face;
        }

        /**
         * Get the face corresponding to the enum value.
         *
         * @return The face corresponding to the enum value
         */
        public Face getFace() {
            return face;
        }
    }

    /**
     * Check the availability of branches at the specified level.
     *
     * @param level The level to check for availability
     * @return An array of available branches at the specified level
     */
    public Branch[] checkHeightAvailability(Level level, boolean useVision) {
        List<Branch> branches = new ArrayList<>();
        for (Face face : faces) {
            if (face.getSelected()) {
                Branch rightBranch = face.getBranch(Side.RIGHT);
                Branch leftBranch = face.getBranch(Side.LEFT);
                if (useVision) {
                    if (!rightBranch.getCoralStatus(level)) {
                        branches.add(rightBranch);
                    }
                    if (!leftBranch.getCoralStatus(level)) {
                        branches.add(leftBranch);
                    }
                } else {
                    branches.add(leftBranch);
                    branches.add(rightBranch);
                }
            }
        }
        return branches.toArray(new Branch[branches.size()]);
    }

    /**
     * Get the closest branch to the current pose at the specified level.
     *
     * @param currentPose The current pose
     * @param level The level to check for availability
     * @return The closest branch to the current pose at the specified level
     */
    public Branch getclosestBranch(Pose2d currentPose, Level level, boolean useVision) {
        Pose2d pose = currentPose;
        if (Robot.isRedAlliance()) {
            pose = FlippingUtil.flipFieldPose(currentPose);
        }
        Level currentLevel = level;
        Branch[] branches = checkHeightAvailability(currentLevel, useVision);
        while (branches.length == 0) {
            currentLevel = getLesserLevel(currentLevel);
            branches = checkHeightAvailability(currentLevel, useVision);
        }
        Branch closestBranch = null;
        double minDistance = Double.MAX_VALUE;
        for (Branch branch : branches) {
            double distance = PoseUtil.getDistance(pose, branch.getPose());
            try {
                System.out.println("Face: " + branch.getFace().getName() + " Side: " + branch.getSide() + " Distance: "
                        + distance);
            } catch (Exception e) {
                System.out.println("Error getting face name: " + e.getMessage());
            }
            if (distance < minDistance) {
                minDistance = distance;
                closestBranch = branch;
            }
        }
        try {
            System.out.println("Face: " + closestBranch.getFace().getName() + " Side: " + closestBranch.getSide());
        } catch (Exception e) {
            System.out.println("Error getting face name: " + e.getMessage());
        }
        return closestBranch;
    }

    /**
     * Get the closest face to the current pose.
     *
     * @param currentPose The current pose
     * @return The closest face to the current pose
     */
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
            availablefaces.add(FaceEnum.FRONT.getFace());
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

    /**
     * Check if the reef is full.
     *
     * @return True if the reef is full, false otherwise
     */
    public boolean isReefFull() {
        for (Face face : faces) {
            if (face.getSelected() && (!face.leftBranch.isFull() || !face.rightBranch.isFull())) {
                return false;
            }
        }
        return true;
    }

    // Pose2d constants representing the positions and orientations of the branches on each face
    private static final Pose2d FL_RIGHT = new Pose2d(3.703, 5.06, Rotation2d.fromDegrees(-60));
    private static final Pose2d FL_LEFT = new Pose2d(3.987, 5.224, Rotation2d.fromDegrees(-60));
    private static final Pose2d F_LEFT = new Pose2d(3.2, 4.19, Rotation2d.fromDegrees(0));
    private static final Pose2d F_RIGHT = new Pose2d(3.2, 3.862, Rotation2d.fromDegrees(0));
    private static final Pose2d FR_LEFT = new Pose2d(3.703, 2.992, Rotation2d.fromDegrees(60));
    private static final Pose2d FR_RIGHT = new Pose2d(3.987, 2.828, Rotation2d.fromDegrees(60));
    private static final Pose2d BL_LEFT = new Pose2d(5.276, 5.06, Rotation2d.fromDegrees(-120));
    private static final Pose2d BL_RIGHT = new Pose2d(4.992, 5.224, Rotation2d.fromDegrees(-120));
    private static final Pose2d B_LEFT = new Pose2d(5.778, 3.862, Rotation2d.fromDegrees(180));
    private static final Pose2d B_RIGHT = new Pose2d(5.778, 4.19, Rotation2d.fromDegrees(180));
    private static final Pose2d BR_LEFT = new Pose2d(4.992, 2.828, Rotation2d.fromDegrees(120));
    private static final Pose2d BR_RIGHT = new Pose2d(5.276, 2.992, Rotation2d.fromDegrees(120));

    /**
     * Get the lesser level compared to the given level.
     *
     * @param priority The current level
     * @return The lesser level
     */
    public static Level getLesserLevel(Level priority) {
        if (priority == Level.L4) {
            return Level.L3;
        } else if (priority == Level.L3) {
            return Level.L2;
        } else if (priority == Level.L2) {
            return Level.L1;
        } else if (priority == Level.L1) {
            return Level.L4;
        } else {
            return Level.L3;
        }
    }

    /**
     * Get the face corresponding to the specified enum value.
     *
     * @param face The enum value representing the face
     * @return The face corresponding to the enum value
     */
    public Face getReefSide(FaceEnum face) {
        if (face == FaceEnum.FRONT) {
            return faces[0];
        } else if (face == FaceEnum.FRONT_LEFT) {
            return faces[1];
        } else if (face == FaceEnum.BACK_LEFT) {
            return faces[2];
        } else if (face == FaceEnum.BACK) {
            return faces[3];
        } else if (face == FaceEnum.BACK_RIGHT) {
            return faces[4];
        } else if (face == FaceEnum.FRONT_RIGHT) {
            return faces[5];
        } else return faces[0];
    }
}
