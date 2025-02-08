package frc.robot.subsystems.remover;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.remover.RemoverIO.RemoverIOInputs;

public class RemoverIOSpark implements RemoverIO {
    private SparkMax rollerMotor;
    private SparkMax pivotMotor;
    private AbsoluteEncoder throughBore;

    private PIDController PIDController =
            new PIDController(RemoverConstants.REMOVER_KP, RemoverConstants.REMOVER_KI, RemoverConstants.REMOVER_KD);

    public RemoverIOSpark() {
        rollerMotor = new SparkMax(RemoverConstants.REMOVER_TURNING_CANID, MotorType.kBrushless);
        pivotMotor = new SparkMax(RemoverConstants.REMOVER_ROTATING_CANID, MotorType.kBrushless);
        throughBore = pivotMotor.getAbsoluteEncoder();
    }

    @Override
    public void setRemoverRollerPositionRad(double radians) {
        PIDController.setSetpoint(radians);
        pivotMotor.set(PIDController.calculate(throughBore.getPosition()));
    }

    @Override
    public void setRemoverRollerSpeed(double speed) {
        rollerMotor.set(speed);
    }

    @Override
    public void updateInputs(RemoverIOInputs inputs) {
        inputs.RemoverPivotPositionRad = throughBore.getPosition();
        inputs.RemoverPivotVelocity = throughBore.getVelocity();
        inputs.RemoverRollerVelocity = rollerMotor.getEncoder().getVelocity();
        inputs.RemoverPivotAppliedVolts = pivotMotor.getAppliedOutput();
        inputs.RemoverRollerVoltage = rollerMotor.getAppliedOutput();
        inputs.RemoverPivotAmps = pivotMotor.getOutputCurrent();
        inputs.RemoverRollerAmps = rollerMotor.getOutputCurrent();
    }
}
