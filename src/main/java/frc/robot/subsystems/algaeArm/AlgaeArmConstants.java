package frc.robot.subsystems.algaeArm;

import edu.wpi.first.math.util.Units;

public class AlgaeArmConstants {
    public static final int PIVOT_MOTOR_CANID = 1;
    public static final int ROLLER_MOTOR_CANID = 2;

    public static final double ROLLERS_SPEED = 1.0;
    public static final double PIVOT_SPEED = 1.0;

    public static final double ALGAE_ARM_KP = 0.0;
    public static final double ALGAE_ARM_KI = 0.0;
    public static final double ALGAE_ARM_KD = 0.0;

    public static final double SIM_ALGAE_ARM_KP = 0.0;
    public static final double SIM_ALGAE_ARM_KI = 0.0;
    public static final double SIM_ALGAE_ARM_KD = 0.0;

    // Arm positions
    public static final double INTAKE_POSITION = Units.degreesToRadians(45);
    public static final double HOLD_POSITION = Units.degreesToRadians(10);
    public static final double INSIDE_POSITION = Units.degreesToRadians(0);
}
