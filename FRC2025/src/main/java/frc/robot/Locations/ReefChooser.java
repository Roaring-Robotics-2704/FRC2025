package frc.robot.Locations;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ReefChooser {
    SendableChooser<Pose2d> reefPosChooser = new SendableChooser<>();

    public ReefChooser(){
      reefPosChooser.setDefaultOption("FLL",ReefLocations.FL_Left);
      reefPosChooser.addOption("FLR",ReefLocations.FL_Right);
      reefPosChooser.addOption("FRL", ReefLocations.FR_Left);
      reefPosChooser.addOption("FRR",ReefLocations.FR_Right);
      reefPosChooser.addOption("FL",ReefLocations.F_Left);
      reefPosChooser.addOption("FR",ReefLocations.F_Right);
      reefPosChooser.addOption("BLL", ReefLocations.BL_Left);
      reefPosChooser.addOption("BLR",ReefLocations.BL_Right);
      reefPosChooser.addOption("BRL",ReefLocations.BR_Left);
      reefPosChooser.addOption("BRR",ReefLocations.BR_Right);
      reefPosChooser.addOption("BL",ReefLocations.B_Left);
      reefPosChooser.addOption("BR",ReefLocations.B_Right);
      SmartDashboard.putData(reefPosChooser);
    }
    public Pose2d getReefPose() {
      return reefPosChooser.getSelected();
  }
}
