package frc.robot.subsystems.algaeArm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.algaeArm.AlgaeArmIO.AlgaeArmIOInputs;

public class AlgaeArmIOSpark implements AlgaeArmIO {
    private SparkMax rollerMotor;
    private SparkMax pivotMotor;
    private AbsoluteEncoder throughBore;

    private PIDController PIDController = new PIDController(
            AlgaeArmConstants.ALGAE_ARM_KP, AlgaeArmConstants.ALGAE_ARM_KI, AlgaeArmConstants.ALGAE_ARM_KD);

    public AlgaeArmIOSpark() {

        rollerMotor = new SparkMax(AlgaeArmConstants.ROLLER_MOTOR_CANID, MotorType.kBrushless);
        pivotMotor = new SparkMax(AlgaeArmConstants.PIVOT_MOTOR_CANID, MotorType.kBrushless);
        throughBore = pivotMotor.getAbsoluteEncoder();
    }

    @Override
    public void setAlgaeArmPosition(double radians) {
        pivotMotor.set(radians);
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
