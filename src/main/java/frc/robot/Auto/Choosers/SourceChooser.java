// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Auto.Choosers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Auto.Constants.SourceLocations;

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
        SmartDashboard.putData(sourcePosChooser);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    public Pose2d getSourcePose() {
        return sourcePosChooser.getSelected();
    }
}
