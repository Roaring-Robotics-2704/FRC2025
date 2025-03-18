package frc.robot.auto.reef;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
    static Face[] faces = new Face[6]; // Array to hold the six faces of the reef
    private static SendableChooser<Face> chooser = new SendableChooser<>(); // Chooser for selecting a face

    /** Private constructor to initialize the faces with their respective positions and orientations. */
    public Reef() {
        faces[0] = new Face(F_LEFT, F_RIGHT, FaceEnum.FRONT); // Initializing face 0
        faces[1] = new Face(FL_LEFT, FL_RIGHT, FaceEnum.FRONT_LEFT); // Initializing face 1
        faces[2] = new Face(BL_LEFT, BL_RIGHT, FaceEnum.BACK_LEFT); // Initializing face 2
        faces[3] = new Face(B_LEFT, B_RIGHT, FaceEnum.BACK); // Initializing face 3
        faces[4] = new Face(BR_LEFT, BR_RIGHT, FaceEnum.BACK_RIGHT); // Initializing face 4
        faces[5] = new Face(FR_LEFT, FR_RIGHT, FaceEnum.FRONT_RIGHT); // Initializing face 5
    }

    /** The Face class represents a face of the reef, containing two branches and a selection status. */
    public class Face {
        Branch rightBranch; // Right branch of the face
        Branch leftBranch; // Left branch of the face
        FaceEnum faceEnum; // Type of the face

        /**
         * Constructor to initialize the face with left and right branch poses and its type.
         *
         * @param leftPose Pose of the left branch
         * @param rightPose Pose of the right branch
         * @param face Type of the face
         */
        public Face(Pose2d leftPose, Pose2d rightPose, FaceEnum face) {
            rightBranch = new Branch(Side.RIGHT, rightPose, this); // Initializing right branch
            leftBranch = new Branch(Side.LEFT, leftPose, this); // Initializing left branch
            this.faceEnum = face; // Setting the face type
        }

        /**
         * Get the branch on the specified side.
         *
         * @param side Side of the branch (LEFT or RIGHT)
         * @return The branch on the specified side
         */
        public Branch getBranch(Side side) {
            if (side == Side.RIGHT) { // If side is RIGHT
                return rightBranch; // Return right branch
            } else {
                return leftBranch; // Return left branch
            }
        }

        /**
         * Get the name of the face.
         *
         * @return Name of the face
         */
        public String getName() {

            return faceEnum.toString(); // Return name of the face
        }
    }

    /** Enum representing the different faces of the reef. */
    public enum FaceEnum {
        FRONT(faces[0]), // Front face
        FRONT_LEFT(faces[1]), // Front left face
        FRONT_RIGHT(faces[2]), // Front right face
        BACK_LEFT(faces[3]), // Back left face
        BACK_RIGHT(faces[4]), // Back right face
        BACK(faces[5]); // Back face

        private final Face face; // Face corresponding to the enum value

        FaceEnum(Face face) {
            this.face = face; // Setting the face
        }

        /**
         * Get the face corresponding to the enum value.
         *
         * @return The face corresponding to the enum value
         */
        public Face getFace() {
            return face; // Return the face
        }
    }

    /**
     * Check the availability of branches at the specified level.
     *
     * @param level The level to check for availability
     * @param useVision Whether to use vision for checking availability
     * @return An array of available branches at the specified level
     */
    public Branch[] checkHeightAvailability(Level level, boolean useVision) {
        List<Branch> branches = new ArrayList<>(); // List to hold available branches
        for (Face face : faces) { // Iterate through faces
            Branch rightBranch = face.getBranch(Side.RIGHT); // Get right branch
            Branch leftBranch = face.getBranch(Side.LEFT); // Get left branch
            if (useVision) { // If using vision
                if (!rightBranch.getCoralStatus(level)) { // If right branch is available
                    branches.add(rightBranch); // Add right branch to list
                }
                if (!leftBranch.getCoralStatus(level)) { // If left branch is available
                    branches.add(leftBranch); // Add left branch to list
                }
            } else {
                branches.add(leftBranch); // Add left branch to list
                branches.add(rightBranch); // Add right branch to list
            }
        }
        return branches.toArray(new Branch[branches.size()]); // Return array of available branches
    }

    /**
     * Get the closest branch to the current pose at the specified level.
     *
     * @param currentPose The current pose
     * @param level The level to check for availability
     * @param useVision Whether to use vision for checking availability
     * @return The closest branch to the current pose at the specified level
     */
    public Branch getclosestBranch(Pose2d currentPose, Level level, boolean useVision) {
        Pose2d pose = currentPose; // Set pose to current pose
        if (Robot.isRedAlliance()) { // If robot is in red alliance
            pose = FlippingUtil.flipFieldPose(currentPose); // Flip field pose
        }
        Level currentLevel = level; // Set current level to specified level
        Branch[] branches = checkHeightAvailability(currentLevel, useVision); // Check height availability
        while (branches.length == 0) { // While no branches are available
            currentLevel = getLesserLevel(currentLevel); // Get lesser level
            branches = checkHeightAvailability(currentLevel, useVision); // Check height availability
        }
        Branch closestBranch = null; // Initialize closest branch
        double minDistance = Double.MAX_VALUE; // Initialize minimum distance
        for (Branch branch : branches) { // Iterate through branches
            double distance = PoseUtil.getDistance(pose, branch.getPose()); // Get distance to branch
            // try {
            //     System.out.println("Face: " + branch.getFace().getName() + " Side: " + branch.getSide() + " Distance:
            // "
            //             + distance); // Print face, side, and distance
            // } catch (Exception e) {
            //     System.out.println("Error getting face name: " + e.getMessage()); // Print error message
            // }
            if (distance < minDistance) { // If distance is less than minimum distance
                minDistance = distance; // Set minimum distance
                closestBranch = branch; // Set closest branch
            }
        }
        // try {
        //     System.out.println("Face: " + closestBranch.getFace().getName() + " Side: "
        //             + closestBranch.getSide()); // Print closest face and side
        // } catch (Exception e) {
        //     System.out.println("Error getting face name: " + e.getMessage()); // Print error message
        // }
        return closestBranch; // Return closest branch
    }

    /**
     * Get the closest face to the current pose.
     *
     * @param currentPose The current pose
     * @return The closest face to the current pose
     */
    public Face getclosestFace(Pose2d currentPose) {
        if (Robot.isRedAlliance()) { // If robot is in red alliance
            currentPose = FlippingUtil.flipFieldPose(currentPose); // Flip field pose
        }
        List<Face> availablefaces = new ArrayList<>(); // List to hold available faces
        for (Face face : faces) { // Iterate through faces
            availablefaces.add(face); // Add face to list
        }
        if (availablefaces.isEmpty()) { // If no faces are available
            availablefaces.add(FaceEnum.FRONT.getFace()); // Add front face to list
        }
        Face closestFace = null; // Initialize closest face
        double minDistance = Double.MAX_VALUE; // Initialize minimum distance
        for (Face face : availablefaces) { // Iterate through available faces
            double distance = currentPose
                    .getTranslation()
                    .getDistance(PoseUtil.averagePose(face.leftBranch.getPose(), face.rightBranch.getPose())
                            .getTranslation()); // Get distance to face
            if (distance < minDistance) { // If distance is less than minimum distance
                minDistance = distance; // Set minimum distance
                closestFace = face; // Set closest face
            }
        }

        return closestFace; // Return closest face
    }

    /**
     * Check if the reef is full.
     *
     * @return True if the reef is full, false otherwise
     */
    public boolean isReefFull() {
        for (Face face : faces) { // Iterate through faces
            if ((face.leftBranch.isFull() && face.rightBranch.isFull())) { // If face is selected and not full

                return false; // Return false if there is still open space
            }
        }
        return true; // Return true if reef is full
    }

    // Pose2d constants representing the positions and orientations of the branches
    // on each face
    private static final Pose2d FL_RIGHT = new Pose2d(3.703, 5.06, Rotation2d.fromDegrees(-60)); // Pose of front left
    // right branch
    private static final Pose2d FL_LEFT = new Pose2d(3.987, 5.224, Rotation2d.fromDegrees(-60)); // Pose of front left
    // left branch
    private static final Pose2d F_LEFT = new Pose2d(3.2, 4.19, Rotation2d.fromDegrees(0)); // Pose of front left branch
    private static final Pose2d F_RIGHT = new Pose2d(3.2, 3.862, Rotation2d.fromDegrees(0)); // Pose of front right
    // branch
    private static final Pose2d FR_LEFT = new Pose2d(3.703, 2.992, Rotation2d.fromDegrees(60)); // Pose of front right
    // left branch
    private static final Pose2d FR_RIGHT = new Pose2d(3.987, 2.828, Rotation2d.fromDegrees(60)); // Pose of front right
    // right branch
    private static final Pose2d BL_LEFT = new Pose2d(5.276, 5.06, Rotation2d.fromDegrees(-120)); // Pose of back left
    // left branch
    private static final Pose2d BL_RIGHT = new Pose2d(4.992, 5.224, Rotation2d.fromDegrees(-120)); // Pose of back left
    // right branch
    private static final Pose2d B_LEFT = new Pose2d(5.778, 3.862, Rotation2d.fromDegrees(180)); // Pose of back left
    // branch
    private static final Pose2d B_RIGHT = new Pose2d(5.778, 4.19, Rotation2d.fromDegrees(180)); // Pose of back right
    // branch
    private static final Pose2d BR_LEFT = new Pose2d(4.992, 2.828, Rotation2d.fromDegrees(120)); // Pose of back right
    // left branch
    private static final Pose2d BR_RIGHT = new Pose2d(5.276, 2.992, Rotation2d.fromDegrees(120)); // Pose of back right
    // right branch

    /**
     * Get the lesser level compared to the given level.
     *
     * @param priority The current level
     * @return The lesser level
     */
    public static Level getLesserLevel(Level priority) {
        if (priority == Level.L4) { // If current level is L4
            return Level.L3; // Return L3
        } else if (priority == Level.L3) { // If current level is L3
            return Level.L2; // Return L2
        } else if (priority == Level.L2) { // If current level is L2
            return Level.L1; // Return L1
        } else if (priority == Level.L1) { // If current level is L1
            return Level.L4; // Return L4
        } else {
            return Level.L3; // Return L3
        }
    }

    /**
     * Get the face corresponding to the specified enum value.
     *
     * @param face The enum value representing the face
     * @return The face corresponding to the enum value
     */
    public Face getReefSide(FaceEnum face) {
        if (face == FaceEnum.FRONT) { // If face is FRONT
            return faces[0]; // Return face 0
        } else if (face == FaceEnum.FRONT_LEFT) { // If face is FRONT_LEFT
            return faces[1]; // Return face 1
        } else if (face == FaceEnum.BACK_LEFT) { // If face is BACK_LEFT
            return faces[2]; // Return face 2
        } else if (face == FaceEnum.BACK) { // If face is BACK
            return faces[3]; // Return face 3
        } else if (face == FaceEnum.BACK_RIGHT) { // If face is BACK_RIGHT
            return faces[4]; // Return face 4
        } else if (face == FaceEnum.FRONT_RIGHT) { // If face is FRONT_RIGHT
            return faces[5]; // Return face 5
        } else return faces[0]; // Return face 0
    }
}
