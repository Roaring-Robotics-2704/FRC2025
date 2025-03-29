package frc.robot.subsystems.remover;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class RemoverConstants {
    public static final int REMOVER_CANID = 16;

    public static final double ROLLER_SPEED = 0.2;
    public static final double CURRENT_LIMIT = 15;
    public static final double LOWER_HEIGHT =
            ElevatorConstants.L2_HEIGHT + Units.inchesToMeters(3); // TODO: Change this to the correct value
    public static final double UPPER_HEIGHT =
            ElevatorConstants.L3_HEIGHT + Units.inchesToMeters(0); // TODO: Change this to the correct value
    public static final double HEIGHT_OFFSET = -Units.inchesToMeters(1); // TODO: Change this to the correct value
}
