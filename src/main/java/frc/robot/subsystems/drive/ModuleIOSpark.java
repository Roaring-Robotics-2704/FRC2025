package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.BACK_LEFT_DRIVE_CAN_ID;
import static frc.robot.subsystems.drive.DriveConstants.BACK_LEFT_TURN_CAN_ID;
import static frc.robot.subsystems.drive.DriveConstants.BACK_RIGHT_DRIVE_CAN_ID;
import static frc.robot.subsystems.drive.DriveConstants.BACK_RIGHT_TURN_CAN_ID;
import static frc.robot.subsystems.drive.DriveConstants.DRIVE_CURRENT_LIMIT;
import static frc.robot.subsystems.drive.DriveConstants.DRIVE_KD;
import static frc.robot.subsystems.drive.DriveConstants.DRIVE_KP;
import static frc.robot.subsystems.drive.DriveConstants.DRIVE_KS;
import static frc.robot.subsystems.drive.DriveConstants.DRIVE_KV;
import static frc.robot.subsystems.drive.DriveConstants.DRIVE_POSITION_FACTOR;
import static frc.robot.subsystems.drive.DriveConstants.DRIVE_VELOCITY_FACTOR;
import static frc.robot.subsystems.drive.DriveConstants.FRONT_LEFT_DRIVE_CAN_ID;
import static frc.robot.subsystems.drive.DriveConstants.FRONT_LEFT_TURN_CAN_ID;
import static frc.robot.subsystems.drive.DriveConstants.FRONT_RIGHT_DRIVE_CAN_ID;
import static frc.robot.subsystems.drive.DriveConstants.FRONT_RIGHT_TURN_CAN_ID;
import static frc.robot.subsystems.drive.DriveConstants.ODOMETRY_FREQUENCY;
import static frc.robot.subsystems.drive.DriveConstants.TURN_CURRENT_LIMIT;
import static frc.robot.subsystems.drive.DriveConstants.TURN_INVERTED;
import static frc.robot.subsystems.drive.DriveConstants.TURN_KD;
import static frc.robot.subsystems.drive.DriveConstants.TURN_KP;
import static frc.robot.subsystems.drive.DriveConstants.TURN_MAX_INPUT;
import static frc.robot.subsystems.drive.DriveConstants.TURN_MIN_INPUT;
import static frc.robot.subsystems.drive.DriveConstants.TURN_POSITION_FACTOR;
import static frc.robot.subsystems.drive.DriveConstants.TURN_VELOCITY_FACTOR;
import static frc.robot.subsystems.drive.DriveConstants.backLeftZeroRotation;
import static frc.robot.subsystems.drive.DriveConstants.backRightZeroRotation;
import static frc.robot.subsystems.drive.DriveConstants.frontLeftZeroRotation;
import static frc.robot.subsystems.drive.DriveConstants.frontRightZeroRotation;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Queue;
import java.util.function.DoubleSupplier;

/**
 * Module IO implementation for Spark Flex drive motor controller, Spark Max turn motor controller, and duty cycle
 * absolute encoder.
 */
public class ModuleIOSpark implements ModuleIO {
    private final Rotation2d zeroRotation;

    // Hardware objects
    private final SparkBase driveSpark;
    private final SparkBase turnSpark;
    private final RelativeEncoder driveEncoder;
    private final AbsoluteEncoder turnEncoder;

    // Closed loop controllers
    private final SparkClosedLoopController driveController;
    private final SparkClosedLoopController turnController;

    // Queue inputs from odometry thread
    private final Queue<Double> timestampQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> turnPositionQueue;

    // Connection debouncers
    private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnConnectedDebounce = new Debouncer(0.5);

    public ModuleIOSpark(int module) {
        zeroRotation = switch (module) {
            case 0 -> frontLeftZeroRotation;
            case 1 -> frontRightZeroRotation;
            case 2 -> backLeftZeroRotation;
            case 3 -> backRightZeroRotation;
            default -> new Rotation2d();};
        driveSpark = new SparkMax(
                switch (module) {
                    case 0 -> FRONT_LEFT_DRIVE_CAN_ID;
                    case 1 -> FRONT_RIGHT_DRIVE_CAN_ID;
                    case 2 -> BACK_LEFT_DRIVE_CAN_ID;
                    case 3 -> BACK_RIGHT_DRIVE_CAN_ID;
                    default -> 0;
                },
                MotorType.kBrushless);
        turnSpark = new SparkMax(
                switch (module) {
                    case 0 -> FRONT_LEFT_TURN_CAN_ID;
                    case 1 -> FRONT_RIGHT_TURN_CAN_ID;
                    case 2 -> BACK_LEFT_TURN_CAN_ID;
                    case 3 -> BACK_RIGHT_TURN_CAN_ID;
                    default -> 0;
                },
                MotorType.kBrushless);
        driveEncoder = driveSpark.getEncoder();
        turnEncoder = turnSpark.getAbsoluteEncoder();
        driveController = driveSpark.getClosedLoopController();
        turnController = turnSpark.getClosedLoopController();

        // Configure drive motor
        var driveConfig = new SparkMaxConfig();
        driveConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(DRIVE_CURRENT_LIMIT)
                .voltageCompensation(12.0);
        driveConfig
                .encoder
                .positionConversionFactor(DRIVE_POSITION_FACTOR)
                .velocityConversionFactor(DRIVE_VELOCITY_FACTOR)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);
        driveConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(
                        DRIVE_KP, 0.0,
                        DRIVE_KD, 0.0);
        driveConfig
                .signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs((int) (1000.0 / ODOMETRY_FREQUENCY))
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        tryUntilOk(
                driveSpark,
                5,
                () -> driveSpark.configure(
                        driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        tryUntilOk(driveSpark, 5, () -> driveEncoder.setPosition(0.0));

        // Configure turn motor
        var turnConfig = new SparkMaxConfig();
        turnConfig
                .inverted(TURN_INVERTED)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(TURN_CURRENT_LIMIT)
                .voltageCompensation(12.0);
        turnConfig
                .absoluteEncoder
                .inverted(TURN_INVERTED)
                .positionConversionFactor(TURN_POSITION_FACTOR)
                .velocityConversionFactor(TURN_VELOCITY_FACTOR)
                .averageDepth(2);
        turnConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(TURN_MIN_INPUT, TURN_MAX_INPUT)
                .pidf(TURN_KP, 0.0, TURN_KD, 0.0);
        turnConfig
                .signals
                .absoluteEncoderPositionAlwaysOn(true)
                .absoluteEncoderPositionPeriodMs((int) (1000.0 / ODOMETRY_FREQUENCY))
                .absoluteEncoderVelocityAlwaysOn(true)
                .absoluteEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        tryUntilOk(
                turnSpark,
                5,
                () -> turnSpark.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        // Create odometry queues
        timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
        drivePositionQueue = SparkOdometryThread.getInstance().registerSignal(driveSpark, driveEncoder::getPosition);
        turnPositionQueue = SparkOdometryThread.getInstance().registerSignal(turnSpark, turnEncoder::getPosition);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Update drive inputs
        sparkStickyFault = false;
        ifOk(driveSpark, driveEncoder::getPosition, value -> inputs.drivePositionRad = value);
        ifOk(driveSpark, driveEncoder::getVelocity, value -> inputs.driveVelocityRadPerSec = value);
        ifOk(
                driveSpark,
                new DoubleSupplier[] {driveSpark::getAppliedOutput, driveSpark::getBusVoltage},
                values -> inputs.driveAppliedVolts = values[0] * values[1]);
        ifOk(driveSpark, driveSpark::getOutputCurrent, value -> inputs.driveCurrentAmps = value);
        inputs.driveConnected = driveConnectedDebounce.calculate(!sparkStickyFault);

        // Update turn inputs
        sparkStickyFault = false;
        ifOk(
                turnSpark,
                turnEncoder::getPosition,
                value -> inputs.turnPosition = new Rotation2d(value).minus(zeroRotation));
        ifOk(turnSpark, turnEncoder::getVelocity, value -> inputs.turnVelocityRadPerSec = value);
        ifOk(
                turnSpark,
                new DoubleSupplier[] {turnSpark::getAppliedOutput, turnSpark::getBusVoltage},
                values -> inputs.turnAppliedVolts = values[0] * values[1]);
        ifOk(turnSpark, turnSpark::getOutputCurrent, value -> inputs.turnCurrentAmps = value);
        inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

        // Update odometry inputs
        inputs.odometryTimestamps =
                timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRad =
                drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryTurnPositions = turnPositionQueue.stream()
                .map((Double value) -> new Rotation2d(value).minus(zeroRotation))
                .toArray(Rotation2d[]::new);
        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveSpark.setVoltage(output);
    }

    @Override
    public void setTurnOpenLoop(double output) {
        turnSpark.setVoltage(output);
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        double ffVolts = DRIVE_KS * Math.signum(velocityRadPerSec) + DRIVE_KV * velocityRadPerSec;
        driveController.setReference(
                velocityRadPerSec, ControlType.kVelocity, ClosedLoopSlot.kSlot0, ffVolts, ArbFFUnits.kVoltage);
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        double setpoint =
                MathUtil.inputModulus(rotation.plus(zeroRotation).getRadians(), TURN_MIN_INPUT, TURN_MAX_INPUT);
        turnController.setReference(setpoint, ControlType.kPosition);
    }
}
