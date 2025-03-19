// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.outtake;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class OuttakeConstants {

    public static final int OUTTAKE_ID = 15;
    public static final double CURRENT_LIMIT = 15;

    public static final double OUTTAKE_SPEED = -.3; // Change speeds and times (in seconds) when testing physical motors
    public static final double INTAKE_SPEED = -0.3;
    public static final double REVERSE_SPEED = 0.2; // Change speeds and times (in seconds) when testing physical motors
    public static final double ALIGN_SPEED = -0.1;
    public static final double NO_REVERSE_HEIGHT = Units.inchesToMeters(3);
}
