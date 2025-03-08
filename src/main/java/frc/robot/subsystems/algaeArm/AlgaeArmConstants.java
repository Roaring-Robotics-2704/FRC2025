package frc.robot.subsystems.algaeArm;

import edu.wpi.first.math.util.Units;

public class AlgaeArmConstants {
    public static final int PIVOT_MOTOR_CANID = 21;
    public static final int ROLLER_MOTOR_CANID = 23;
    public static final int FOLLOW_MOTOR_CANID = 22;

    public static final double ROLLERS_SPEED = 1.0;
    public static final double PIVOT_SPEED = 1.0;

    public static final double ALGAE_ARM_KP = 1.5;
    public static final double ALGAE_ARM_KI = 0.0;
    public static final double ALGAE_ARM_KD = 0.0;
    public static final double ALGAE_ARM_KS = 0.0;

    public static final double SIM_ALGAE_ARM_KP = 0134E-07;
    public static final double SIM_ALGAE_ARM_KI = 0.0;
    public static final double SIM_ALGAE_ARM_KD = 3726E-06;
    public static final double KS = 0.038773;
    public static final double KG = 0.040264;
    public static final double KV = 0.00036869;

    // Arm positions
    public static final double INTAKE_POSITION = Units.degreesToRadians(323);
    public static final double HOLD_POSITION = Units.degreesToRadians(285);
    public static final double INSIDE_POSITION = Units.degreesToRadians(265);
}
