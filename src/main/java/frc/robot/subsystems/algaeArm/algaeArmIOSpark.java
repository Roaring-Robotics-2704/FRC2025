package frc.robot.subsystems.algaeArm;

import static frc.robot.subsystems.drive.DriveConstants.DRIVE_VELOCITY_FACTOR;
import static frc.robot.subsystems.drive.DriveConstants.TURN_POSITION_FACTOR;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.algaeArm.AlgaeArmConstants.*;
import frc.robot.subsystems.algaeArm.AlgaeArmIO.AlgaeArmIOInputs;
import frc.robot.util.SparkUtil.*;

public class AlgaeArmIOSpark implements AlgaeArmIO {
    private SparkMax rollerMotor;
    private SparkMax pivotMotor;
    private AbsoluteEncoder throughBore;
    private SparkClosedLoopController pivotController;

    private PIDController PIDController = new PIDController(
            AlgaeArmConstants.ALGAE_ARM_KP, AlgaeArmConstants.ALGAE_ARM_KI, AlgaeArmConstants.ALGAE_ARM_KD);

    public AlgaeArmIOSpark() {

        rollerMotor = new SparkMax(AlgaeArmConstants.ROLLER_MOTOR_CANID, MotorType.kBrushless);
        pivotMotor = new SparkMax(AlgaeArmConstants.PIVOT_MOTOR_CANID, MotorType.kBrushless);
        throughBore = pivotMotor.getAbsoluteEncoder();

        pivotController = pivotMotor.getClosedLoopController();

        // Configure drive motor
        var driveConfig = new SparkMaxConfig();
        driveConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12.0);
        driveConfig
                .encoder
                .positionConversionFactor(TURN_POSITION_FACTOR)
                .velocityConversionFactor(DRIVE_VELOCITY_FACTOR)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);
        driveConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pidf(
                        AlgaeArmConstants.ALGAE_ARM_KP, 0.0,
                        AlgaeArmConstants.ALGAE_ARM_KD, 0.0);
        driveConfig
                .signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        tryUntilOk(
                pivotMotor,
                5,
                () -> pivotMotor.configure(
                        driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }

    @Override
    public void setAlgaeArmPosition(double radians) {
        pivotController.setReference(radians, ControlType.kPosition);
    }

    @Override
    public void setRollerSpeed(double speed) {
        rollerMotor.set(speed);
    }

    @Override
    public void updateInputs(AlgaeArmIOInputs inputs) {
        inputs.algaePivotPositionRad = throughBore.getPosition();
        inputs.algaePivotVelocity = throughBore.getVelocity();
        inputs.algaeRollerVelocity = rollerMotor.getEncoder().getVelocity();
        inputs.algaePivotAppliedVolts = pivotMotor.getAppliedOutput();
        inputs.algaeRollerVoltage = rollerMotor.getAppliedOutput();
        inputs.algaePivotAmps = pivotMotor.getOutputCurrent();
        inputs.algaeRollerAmps = rollerMotor.getOutputCurrent();
    }
}
