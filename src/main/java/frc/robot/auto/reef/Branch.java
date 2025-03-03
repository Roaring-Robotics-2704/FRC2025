// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.reef;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.auto.reef.Reef.Face;

/**
 * Represents a branch in the reef with different levels and a pose.
 */
public class Branch {
    private Boolean level4 = false; // Status of level 4
    private Boolean level3 = false; // Status of level 3
    private Boolean level2 = false; // Status of level 2
    private Boolean level1 = false; // Status of level 1
    private Pose2d pose; // Pose of the branch
    Side side = null; // Side of the branch
    Face face; // Face of the branch

    /**
     * Constructs a Branch with the specified side, pose, and face.
     *
     * @param side The side of the branch.
     * @param pose The pose of the branch.
     * @param face The face of the branch.
     */
    public Branch(Side side, Pose2d pose, Face face) {
        this.side = side; // Initialize side
        this.pose = pose; // Initialize pose
        this.face = face; // Initialize face
    }

    /**
     * Represents the side of the branch.
     */
    public enum Side {
        LEFT, // Left side
        RIGHT // Right side
    }

    /**
     * Represents the levels of the branch.
     */
    public enum Level {
        L4, // Level 4
        L3, // Level 3
        L2, // Level 2
        L1 // Level 1
    }

    /**
     * Gets the side of the branch.
     *
     * @return The side of the branch.
     */
    public Side getSide() {
        return side; // Return side
    }

    /**
     * Gets the face of the branch.
     *
     * @return The face of the branch.
     */
    public Face getFace() {
        return face; // Return face
    }

    /**
     * Sets the coral status for a specific level.
     *
     * @param level The level to set the status for.
     * @param status The status to set.
     */
    public void setCoralStatus(Level level, Boolean status) {
        switch (level) { // Switch based on level
            case L4:
                level4 = status; // Set level 4 status
                break;
            case L3:
                level3 = status; // Set level 3 status
                break;
            case L2:
                level2 = status; // Set level 2 status
                break;
            case L1:
                level1 = status; // Set level 1 status
                break;
        }
    }

    /**
     * Gets the coral status for a specific level.
     *
     * @param level The level to get the status for.
     * @return The status of the specified level.
     */
    public Boolean getCoralStatus(Level level) {
        switch (level) { // Switch based on level
            case L4:
                return level4; // Return level 4 status
            case L3:
                return level3; // Return level 3 status
            case L2:
                return level2; // Return level 2 status
            case L1:
                return level1; // Return level 1 status
            default:
                return false; // Default case
        }
    }

    /**
     * Gets the pose of the branch.
     *
     * @return The pose of the branch.
     */
    public Pose2d getPose() {
        return pose; // Return pose
    }

    /**
     * Checks if all levels are full.
     *
     * @return True if all levels are full, false otherwise.
     */
    public boolean isFull() {
        return level4 && level3 && level2 && level1; // Check if all levels are full
    }
}
