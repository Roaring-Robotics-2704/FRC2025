// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.locations;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class ReefLocations {
    // Private constructor to hide the implicit public one
    private ReefLocations() {}
    // Front Left Reef locations
    static final Pose2d FL_RIGHT = new Pose2d(3.703, 5.06, Rotation2d.fromDegrees(-60));
    static final Pose2d FL_LEFT = new Pose2d(3.987, 5.224, Rotation2d.fromDegrees(-60));

    // Front Reef locations
    static final Pose2d F_LEFT = new Pose2d(3.2, 4.19, Rotation2d.fromDegrees(0));
    static final Pose2d F_RIGHT = new Pose2d(3.2, 3.862, Rotation2d.fromDegrees(0));

    // Front Right Reef locations
    static final Pose2d FR_LEFT = new Pose2d(3.703, 2.992, Rotation2d.fromDegrees(60));
    static final Pose2d FR_RIGHT = new Pose2d(3.987, 2.828, Rotation2d.fromDegrees(60));

    // Back Left Reef locations
    static final Pose2d BL_LEFT = new Pose2d(5.276, 5.06, Rotation2d.fromDegrees(-120));
    static final Pose2d BL_RIGHT = new Pose2d(4.992, 5.224, Rotation2d.fromDegrees(-120));

    // Back Reef locations
    static final Pose2d B_LEFT = new Pose2d(5.778, 3.862, Rotation2d.fromDegrees(180));
    static final Pose2d B_RIGHT = new Pose2d(5.778, 4.19, Rotation2d.fromDegrees(180));

    // Back Right Reef locations
    static final Pose2d BR_LEFT = new Pose2d(4.992, 2.828, Rotation2d.fromDegrees(120));
    static final Pose2d BR_RIGHT = new Pose2d(5.276, 2.992, Rotation2d.fromDegrees(120));
    // static final Pose2d BR_CENTER
}
