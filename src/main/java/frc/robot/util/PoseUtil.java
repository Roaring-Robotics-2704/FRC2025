// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

/** Add your docs here. */
public class PoseUtil {
    // Private constructor to prevent instantiation
    private PoseUtil() {}

    public static Pose2d offsetPose(Pose2d originalPose, double relXOffset, double relYOffset) {

        Transform2d transform = new Transform2d(relXOffset, relYOffset, new Rotation2d());

        // Apply the transform to the original pose
        return originalPose.plus(transform);
    }

    public static Pose2d averagePose(Pose2d pose1, Pose2d pose2) {
        double x = (pose1.getX() + pose2.getX()) / 2;
        double y = (pose1.getY() + pose2.getY()) / 2;
        double theta = (pose1.getRotation().getDegrees() + pose2.getRotation().getDegrees()) / 2;
        return new Pose2d(x, y, Rotation2d.fromDegrees(theta));
    }

    public static double getDistance(Pose2d currentPose, Pose2d sourceLeft) {
        return Math.sqrt(Math.pow(currentPose.getX() - sourceLeft.getX(), 2)
                + Math.pow(currentPose.getY() - sourceLeft.getY(), 2));
    }
}
