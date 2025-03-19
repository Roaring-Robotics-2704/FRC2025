package frc.robot.subsystems.algaeArm;

import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;

public class AlgaeArmIOSpark implements AlgaeArmIO {
    private SparkMax rollerMotor;
    private SparkMax pivotMotor;
    private SparkMax pivotFollowMotor;
    private AbsoluteEncoder throughBore;
    private SparkClosedLoopController pivotController;

    public AlgaeArmIOSpark() {

        rollerMotor = new SparkMax(AlgaeArmConstants.ROLLER_MOTOR_CANID, MotorType.kBrushless);
        pivotMotor = new SparkMax(AlgaeArmConstants.PIVOT_MOTOR_CANID, MotorType.kBrushless);
        pivotFollowMotor = new SparkMax(AlgaeArmConstants.FOLLOW_MOTOR_CANID, MotorType.kBrushless);

        throughBore = pivotMotor.getAbsoluteEncoder();

        pivotController = pivotMotor.getClosedLoopController();

        // Configure drive motor
        SparkMaxConfig driveConfig = new SparkMaxConfig();
        SparkMaxConfig followConfig = new SparkMaxConfig();
        SparkMaxConfig rollerConfig = new SparkMaxConfig();
        driveConfig.inverted(true);
        driveConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12.0);
        driveConfig.absoluteEncoder.positionConversionFactor(360).velocityConversionFactor(360);
        driveConfig.absoluteEncoder.zeroCentered(true).inverted(true);

        driveConfig
                .signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        driveConfig.absoluteEncoder.inverted(true);
        // driveConfig.closedLoop.positionWrappingEnabled(true).positionWrappingInputRange(-360, 360);
        // driveConfig.closedLoop.maxMotion.maxAcceleration(0.5).maxVelocity(0.5);
        rollerConfig.idleMode(IdleMode.kBrake);

        tryUntilOk(
                pivotMotor,
                5,
                () -> pivotMotor.configure(
                        driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        followConfig.follow(pivotMotor, false);
        tryUntilOk(
                pivotFollowMotor,
                5,
                () -> pivotFollowMotor.configure(
                        followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        tryUntilOk(
                rollerMotor,
                5,
                () -> rollerMotor.configure(
                        rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }

    @Override
    public void setAlgaeArmPosition(Rotation2d rotation) {
        pivotController.setReference(rotation.getDegrees(), ControlType.kMAXMotionPositionControl);
    }

    @Override
    public void setRollerSpeed(double speed) {
        rollerMotor.set(speed);
    }

    @Override
    public void setAlgaeArmVoltage(double voltage) {
        pivotMotor.setVoltage(voltage);
    }

    @Override
    public void updateInputs(AlgaeArmIOInputs inputs) {
        inputs.algaePivotPositionDeg = throughBore.getPosition();
        inputs.algaePivotVelocity = throughBore.getVelocity();
        inputs.algaeRollerVelocity = rollerMotor.getEncoder().getVelocity();
        inputs.algaePivotAppliedVolts = pivotMotor.getAppliedOutput();
        inputs.algaeRollerVoltage = rollerMotor.getAppliedOutput();
        inputs.algaePivotAmps = pivotMotor.getOutputCurrent();
        inputs.algaeRollerAmps = rollerMotor.getOutputCurrent();
    }
}
