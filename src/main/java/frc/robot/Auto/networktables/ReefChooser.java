// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.networktables;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.constants.ReefLocations;

/** Add your docs here. */
public class ReefChooser {
    SendableChooser<Pose2d> reefPosChooser = new SendableChooser<>();

    // Private constructor to hide the implicit public one
    public ReefChooser() {
        reefPosChooser.setDefaultOption("Front L", ReefLocations.F_LEFT);
        reefPosChooser.addOption("Front R", ReefLocations.F_RIGHT);
        reefPosChooser.addOption("Front Left L", ReefLocations.FL_LEFT);
        reefPosChooser.addOption("Front Left R", ReefLocations.FL_RIGHT);
        reefPosChooser.addOption("Front Right L", ReefLocations.FR_LEFT);
        reefPosChooser.addOption("Front Right R", ReefLocations.FR_RIGHT);
        reefPosChooser.addOption("Back Left L", ReefLocations.BL_LEFT);
        reefPosChooser.addOption("Back Left R", ReefLocations.BL_RIGHT);
        reefPosChooser.addOption("Back Right L", ReefLocations.BR_LEFT);
        reefPosChooser.addOption("Back Right R", ReefLocations.BR_RIGHT);
        reefPosChooser.addOption("Back L", ReefLocations.B_LEFT);
        reefPosChooser.addOption("Back R", ReefLocations.B_RIGHT);
        SmartDashboard.putData("Reef Pos", reefPosChooser);
    }

    public Pose2d getReefPose() {
        return reefPosChooser.getSelected();
    }

    public SendableChooser<Pose2d> getReefChooser() {
        return reefPosChooser;
    }
}
