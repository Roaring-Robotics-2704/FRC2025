// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.source;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.util.PoseUtil;
import java.util.function.Supplier;

public class SourceChooser {
    // Needs to be a supplier to work with getting the closest
    SendableChooser<Supplier<Pose2d>> sourcePosChooser = new SendableChooser<>();

    /** Creates a new SourceChooser. */
    public SourceChooser() {
        // Set default option for the chooser
        sourcePosChooser.setDefaultOption("Auto", SourceLocations.getClosestSource());
        // Add options for different source locations
        sourcePosChooser.addOption("Left Far", () -> SourceLocations.SOURCE_LEFT_FAR);
        sourcePosChooser.addOption("Left Middle", () -> SourceLocations.SOURCE_LEFT);
        sourcePosChooser.addOption("Left Close", () -> SourceLocations.SOURCE_LEFT_CLOSE);
        sourcePosChooser.addOption("Right Close", () -> SourceLocations.SOURCE_RIGHT_CLOSE);
        sourcePosChooser.addOption("Right Middle", () -> SourceLocations.SOURCE_RIGHT);
        sourcePosChooser.addOption("Right Far", () -> SourceLocations.SOURCE_RIGHT_FAR);
        // Put the chooser on the SmartDashboard
        SmartDashboard.putData("Source Pos", sourcePosChooser);
    }

    /**
     * Gets the driver-set source's pose.
     *
     * @return The selected source pose.
     */
    public Pose2d getSourcePose() {
        return sourcePosChooser.getSelected().get();
    }

    /**
     * Gets the closest source to the robot to get coral.
     *
     * @return The closest source pose.
     */
    public Pose2d getClosestSourcePose() {
        return SourceLocations.getClosestSource().get(); // Uses the one from the Source Locations
    }

    public class SourceLocations {
        public SourceLocations() {}

        // Define source locations with specific poses
        public static final Pose2d SOURCE_LEFT = new Pose2d(1.121, 7.025, Rotation2d.fromDegrees(-54));
        public static final Pose2d SOURCE_RIGHT = new Pose2d(1.121, 1.027, Rotation2d.fromDegrees(54));

        // Define offset source locations
        public static final Pose2d SOURCE_LEFT_CLOSE =
                PoseUtil.offsetPose(SOURCE_LEFT, -0.5, 0); // TODO fill in actual values
        public static final Pose2d SOURCE_LEFT_FAR =
                PoseUtil.offsetPose(SOURCE_LEFT, 0.5, 0); // TODO fill in actual values
        public static final Pose2d SOURCE_RIGHT_CLOSE =
                PoseUtil.offsetPose(SOURCE_RIGHT, 0.5, 0); // TODO fill in actual values
        public static final Pose2d SOURCE_RIGHT_FAR =
                PoseUtil.offsetPose(SOURCE_RIGHT, -0.5, 0); // TODO fill in actual values

        /**
         * Gets the closest source to the robot to get coral from.
         *
         * @return A supplier providing the closest source pose.
         */
        public static Supplier<Pose2d> getClosestSource() {
            return () -> (PoseUtil.getDistance(RobotContainer.getBluePose(), SOURCE_LEFT)
                            < PoseUtil.getDistance(RobotContainer.getBluePose(), SOURCE_RIGHT)
                    ? SOURCE_LEFT
                    : SOURCE_RIGHT);
        }
    }
}
