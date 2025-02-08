package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class PoseUtil {
    // Private constructor to prevent instantiation
    private PoseUtil() {}

    public static Pose2d offsetPose(Pose2d originalPose, double relXOffset, double relYOffset) {
        Transform2d transform2d =
                new Transform2d(new Translation2d(relXOffset, relYOffset), originalPose.getRotation());

        // Apply the rotated offset to the original pose's translation
        return originalPose.plus(transform2d);
    }

    public static Pose2d averagePose(Pose2d pose1, Pose2d pose2) {
        double x = (pose1.getX() + pose2.getX()) / 2;
        double y = (pose1.getY() + pose2.getY()) / 2;
        double theta = (pose1.getRotation().getDegrees() + pose2.getRotation().getDegrees()) / 2;
        return new Pose2d(x, y, Rotation2d.fromDegrees(theta));
    }
}
