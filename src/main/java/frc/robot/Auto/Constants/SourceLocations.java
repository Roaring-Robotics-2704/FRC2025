// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Auto.Constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.PoseUtil;

/** Add your docs here. */
public class SourceLocations {
    private SourceLocations() {}
    public static final Pose2d SOURCE_LEFT =
            new Pose2d(1.121, 1.027, Rotation2d.fromDegrees(0)); // TODO fill in actual values
    public static final Pose2d SOURCE_RIGHT =
            new Pose2d(1.121, 1.027, Rotation2d.fromDegrees(0)); // TODO fill in actual values

    public static final Pose2d SOURCE_LEFT_CLOSE =
            PoseUtil.offsetPose(SOURCE_LEFT, -0.5, 0); // TODO fill in actual values
    public static final Pose2d SOURCE_LEFT_FAR = PoseUtil.offsetPose(SOURCE_LEFT, 0.5, 0); // TODO fill in actual values
    public static final Pose2d SOURCE_RIGHT_CLOSE =
            PoseUtil.offsetPose(SOURCE_RIGHT, 0.5, 0); // TODO fill in actual values
    public static final Pose2d SOURCE_RIGHT_FAR =
            PoseUtil.offsetPose(SOURCE_RIGHT, -0.5, 0); // TODO fill in actual values
}
