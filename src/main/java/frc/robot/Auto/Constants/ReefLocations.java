// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

/** Add your docs here. */
public class ReefLocations {
    // Private constructor to hide the implicit public one
    private ReefLocations() {}

    // Priority list of reef heights
    public static final int L1_PRIORITY = 3;
    public static final int L2_PRIORITY = 2;
    public static final int L3_PRIORITY = 1;
    public static final int L4_PRIORITY = 4;

    // Front Left Reef locations
    public static final Pose2d FL_RIGHT = new Pose2d(3.703, 5.06, Rotation2d.fromDegrees(-60));
    public static final Pose2d FL_LEFT = new Pose2d(3.987, 5.224, Rotation2d.fromDegrees(-60));

    protected static final Pose2d[] FL_POSES = {FL_LEFT, FL_RIGHT};
    public static boolean FL_SELECTOR = true;

    // Front Reef locations
    public static final Pose2d F_LEFT = new Pose2d(3.2, 4.19, Rotation2d.fromDegrees(0));
    public static final Pose2d F_RIGHT = new Pose2d(3.2, 3.862, Rotation2d.fromDegrees(0));

    protected static final Pose2d[] F_POSES = {F_LEFT, F_RIGHT};
    public static boolean F_SELECTOR = true;

    // Front Right Reef locations
    public static final Pose2d FR_LEFT = new Pose2d(3.703, 2.992, Rotation2d.fromDegrees(60));
    public static final Pose2d FR_RIGHT = new Pose2d(3.987, 2.828, Rotation2d.fromDegrees(60));

    protected static final Pose2d[] FR_POSES = {FR_LEFT, FR_RIGHT};
    public static boolean FR_SELECTOR = true;

    // Back Left Reef locations
    public static final Pose2d BL_LEFT = new Pose2d(5.276, 5.06, Rotation2d.fromDegrees(-120));
    public static final Pose2d BL_RIGHT = new Pose2d(4.992, 5.224, Rotation2d.fromDegrees(-120));

    protected static final Pose2d[] BL_POSES = {BL_LEFT, BL_RIGHT};
    public static boolean BL_SELECTOR = true;

    // Back Reef locations
    public static final Pose2d B_LEFT = new Pose2d(5.778, 3.862, Rotation2d.fromDegrees(180));
    public static final Pose2d B_RIGHT = new Pose2d(5.778, 4.19, Rotation2d.fromDegrees(180));

    protected static final Pose2d[] B_POSES = {B_LEFT, B_RIGHT};
    public static boolean B_SELECTOR = true;

    // Back Right Reef locations
    public static final Pose2d BR_LEFT = new Pose2d(4.992, 2.828, Rotation2d.fromDegrees(120));
    public static final Pose2d BR_RIGHT = new Pose2d(5.276, 2.992, Rotation2d.fromDegrees(120));

    protected static final Pose2d[] BR_POSES = {BR_LEFT, BR_RIGHT};
    public static boolean BR_SELECTOR = true;

    public enum CoralStatus {
        L1,
        L2,
        L3,
        L4
    }

    private static final Map<CoralStatus, Set<Pose2d>> coralStatusMap = new HashMap<>();

    static {
        for (CoralStatus status : CoralStatus.values()) {
            coralStatusMap.put(status, new HashSet<>());
        }
    }

    public static void setCoralStatus(CoralStatus status, Pose2d pose) {
        if (coralStatusMap.containsKey(status)) {
            coralStatusMap.get(status).add(pose);
        }
    }

    public static Set<Pose2d> getCoralStatusPoses(CoralStatus status) {
        return coralStatusMap.getOrDefault(status, new HashSet<>());
    }

    public static void clearCoralStatus(CoralStatus status) {
        if (coralStatusMap.containsKey(status)) {
            coralStatusMap.get(status).clear();
        }
    }

    public static Pose2d getHighestPriorityPose() {
        Pose2d[] allPoses = {
            FL_LEFT, FL_RIGHT, F_LEFT, F_RIGHT, FR_LEFT, FR_RIGHT, BL_LEFT, BL_RIGHT, B_LEFT, B_RIGHT, BR_LEFT, BR_RIGHT
        };
        CoralStatus[] priorities = {CoralStatus.L3, CoralStatus.L2, CoralStatus.L1, CoralStatus.L4};

        for (CoralStatus status : priorities) {
            for (Pose2d pose : allPoses) {
                if (isPoseSelected(pose)) {
                    return pose;
                }
            }
        }
        return allPoses[0]; // Return the first pose as a fallback
    }

    private static boolean isPoseSelected(Pose2d pose) {
        if (FL_SELECTOR && (pose.equals(FL_LEFT) || pose.equals(FL_RIGHT))) return true;
        if (F_SELECTOR && (pose.equals(F_LEFT) || pose.equals(F_RIGHT))) return true;
        if (FR_SELECTOR && (pose.equals(FR_LEFT) || pose.equals(FR_RIGHT))) return true;
        if (BL_SELECTOR && (pose.equals(BL_LEFT) || pose.equals(BL_RIGHT))) return true;
        if (B_SELECTOR && (pose.equals(B_LEFT) || pose.equals(B_RIGHT))) return true;
        if (BR_SELECTOR && (pose.equals(BR_LEFT) || pose.equals(BR_RIGHT))) return true;
        return false;
    }
}
