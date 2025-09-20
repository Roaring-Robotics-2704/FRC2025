// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;

/** Add your docs here. */
public class ElevatorConstants {
    private ElevatorConstants() {}

    public static final double HEIGHT_TOLERANCE = Units.inchesToMeters(2);
    public static final double MAX_ELEVATOR_VOLTAGE = 12;

    // Motor IDs
    public static final int ELEVATOR_MOTOR_1 = 12;
    public static final int ELEVATOR_MOTOR_2 = 11;
    public static final AnalogInput ANALOG_INPUT = new AnalogInput(0);

    // Encoder ports
    public static final int ENCODER_A = 0;
    public static final int ENCODER_B = 1;
    // Motor Configs
    public static final int CURRENT_LIMIT = 60;
    public static final double PULLEY_RADIUS = Units.inchesToMeters(0.75); // meters
    public static final double GEAR_REDUCTION = 5 / 1;
    public static final DCMotor ELEVATOR_GEARBOX = DCMotor.getNEO(2);
    // Mass
    public static final double CARRIAGE_MASS = Units.lbsToKilograms(15);
    // PID Constants
    public static final double ELEVATOR_KP = 12.0; // 10.5
    public static final double ELEVATOR_KI = 0;
    public static final double ELEVATOR_KD = 0; // 3.1904;

    // Feedforward Constants
    public static final double kS = 0.5;
    public static final double kG = 0.8; // 1.1
    public static final double kV = 3;
    public static final double kA = 0.6; // 1; // 0.23908;

    // Heights
    public static final double OFFSET = Units.inchesToMeters(18);
    // TODO get real values
    public static final double MIN_HEIGHT = Units.inchesToMeters(0); // simulation only
    public static final double MAX_HEIGHT = Units.inchesToMeters(74); // simulation only
    public static final double L1_HEIGHT = Units.inchesToMeters(20) - OFFSET;
    public static final double L2_HEIGHT = Units.inchesToMeters(31) - OFFSET;
    public static final double L3_HEIGHT = Units.inchesToMeters(46) - OFFSET;
    public static final double L4_HEIGHT = Units.inchesToMeters(69) - OFFSET;
    public static final double INTAKE_HEIGHT = Units.inchesToMeters(-0.25);
}
