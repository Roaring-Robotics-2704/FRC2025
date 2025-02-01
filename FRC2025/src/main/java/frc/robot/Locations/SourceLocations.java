package frc.robot.Locations;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.PoseUtil;

/** Add your docs here. */
public class SourceLocations {
    public static final Pose2d SOURCE_LEFT =
            new Pose2d(10, 10, Rotation2d.fromDegrees(0)); // TODO fill in actual values
    public static final Pose2d SOURCE_RIGHT =
            new Pose2d(10, 10, Rotation2d.fromDegrees(0)); // TODO fill in actual values

    public static final Pose2d SOURCE_LEFT_CLOSE =
            PoseUtil.offsetPose(SOURCE_LEFT, -0.5, 0); // TODO fill in actual values
    public static final Pose2d SOURCE_LEFT_FAR = PoseUtil.offsetPose(SOURCE_LEFT, 0.5, 0); // TODO fill in actual values
    public static final Pose2d SOURCE_RIGHT_CLOSE =
            PoseUtil.offsetPose(SOURCE_RIGHT, 0.5, 0); // TODO fill in actual values
    public static final Pose2d SOURCE_RIGHT_FAR =
            PoseUtil.offsetPose(SOURCE_RIGHT, -0.5, 0); // TODO fill in actual values
}
