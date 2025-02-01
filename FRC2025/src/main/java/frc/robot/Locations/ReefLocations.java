package frc.robot.Locations;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.PoseUtil;

public class ReefLocations {
   public static final Pose2d FL_Left = 
    new Pose2d(3.073,5.06,Rotation2d.fromDegrees(-60));
  public static final Pose2d FL_Right = 
    new Pose2d(3.987,5.224,Rotation2d.fromDegrees(-60));

  public static final Pose2d FR_Left =
  new Pose2d(3.703,2.992,Rotation2d.fromDegrees(60));
  public static final Pose2d FR_Right =
  new Pose2d(3.987,2.828,Rotation2d.fromDegrees (60));

  public static final Pose2d F_Left =
  new Pose2d(3.2,4.19,Rotation2d.fromRadians(0));
  public static final Pose2d F_Right =
  new Pose2d(3.2,3.862,Rotation2d.fromDegrees(0));

  public static final Pose2d BL_Left = 
  new Pose2d(5.276,5.06,Rotation2d.fromDegrees(-120));
  public static final Pose2d BL_Right =
  new Pose2d(4.992,5.224,Rotation2d.fromDegrees(-120));

  public static final Pose2d BR_Left=
  new Pose2d(4.992,2.828,Rotation2d.fromDegrees(120));
  public static final Pose2d BR_Right =
  new Pose2d(5.276,2.992,Rotation2d.fromDegrees(120));
  public static final Pose2d B_Left =
  new Pose2d(5.778,3.826,Rotation2d.fromDegrees(180));
  public static final Pose2d B_Right =
  new Pose2d(5.778,4.19,Rotation2d.fromDegrees(180));



}
