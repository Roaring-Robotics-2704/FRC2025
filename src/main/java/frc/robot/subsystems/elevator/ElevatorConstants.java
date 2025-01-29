// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ElevatorConstants {

    // Motor IDs
    public static final int ELEVATOR_MOTOR_1 = 12;
    public static final int ELEVATOR_MOTOR_2 = 13;

    // Encoder ports
    public static final int ENCODER_A = 0;
    public static final int ENCODER_B = 1;
    // Motor Configs
    public static final int CURRENT_LIMIT = 40;
    public static final double PULLEY_RADIUS = Units.inchesToMeters(1.5); // meters
    public static final double GEAR_REDUCTION = (45.0 * 22.0) / (14.0 * 15.0); // TODO get real values
    public static final DCMotor ELEVATOR_GEARBOX = DCMotor.getNEO(2);
    public static final double DISTANCE_PER_PULSE = 2.0 * Math.PI * PULLEY_RADIUS / 4096;

    // Mass
    public static final double CARRIAGE_MASS = Units.lbsToKilograms(50);
    // PID Constants
    public static final double ELEVATOR_KP = 0.0;
    public static final double ELEVATOR_KI = 0.0;
    public static final double ELEVATOR_KD = 0.0;
    public static final double ELEVATOR_KS = 0.0;
    public static final double ELEVATOR_KV = 0.0;
    public static final double ELEVATOR_KG = 0.0;
    public static final double ELEVATOR_KA = 0.0;

    // Heights
    // TODO get real values
    public static final double MIN_HEIGHT = 0.5;
    public static final double MAX_HEIGHT = 74;
    public static final double L1_HEIGHT = 18;
    public static final double L2_HEIGHT = 32;
    public static final double L3_HEIGHT = 47.5;
    public static final double L4_HEIGHT = 72;
}
