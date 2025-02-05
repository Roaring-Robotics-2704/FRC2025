// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.source;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.PoseUtil;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SourceChooser {
    SendableChooser<Pose2d> sourcePosChooser = new SendableChooser<>();
    /** Creates a new SourceChooser. */
    public SourceChooser() {
        sourcePosChooser.setDefaultOption("Left Far", SourceLocations.SOURCE_LEFT_FAR);
        sourcePosChooser.addOption("Left Middle", SourceLocations.SOURCE_LEFT);
        sourcePosChooser.addOption("Left Close", SourceLocations.SOURCE_LEFT_CLOSE);
        sourcePosChooser.addOption("Right Close", SourceLocations.SOURCE_RIGHT_CLOSE);
        sourcePosChooser.addOption("Right Middle", SourceLocations.SOURCE_RIGHT);
        sourcePosChooser.addOption("Right Far", SourceLocations.SOURCE_RIGHT_FAR);
        SmartDashboard.putData("Source Pos", sourcePosChooser);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    public Pose2d getSourcePose() {
        return sourcePosChooser.getSelected();
    }

    public SendableChooser<Pose2d> getSourceChooser() {
        return sourcePosChooser;
    }

    public class SourceLocations {
        private SourceLocations() {}

        public static final Pose2d SOURCE_LEFT =
                new Pose2d(1.121, 7.025, Rotation2d.fromDegrees(-54)); // TODO fill in actual values
        public static final Pose2d SOURCE_RIGHT = new Pose2d(1.121, 1.027, Rotation2d.fromDegrees(54));

        public static final Pose2d SOURCE_LEFT_CLOSE =
                PoseUtil.offsetPose(SOURCE_LEFT, -0.5, 0); // TODO fill in actual values
        public static final Pose2d SOURCE_LEFT_FAR =
                PoseUtil.offsetPose(SOURCE_LEFT, 0.5, 0); // TODO fill in actual values
        public static final Pose2d SOURCE_RIGHT_CLOSE =
                PoseUtil.offsetPose(SOURCE_RIGHT, 0.5, 0); // TODO fill in actual values
        public static final Pose2d SOURCE_RIGHT_FAR =
                PoseUtil.offsetPose(SOURCE_RIGHT, -0.5, 0); // TODO fill in actual values
    }
}
