package frc.robot.subsystems.remover;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class RemoverIOSpark implements RemoverIO {
    private SparkMax rollerMotor;

    public RemoverIOSpark() {
        rollerMotor = new SparkMax(RemoverConstants.REMOVER_TURNING_CANID, MotorType.kBrushless);
    }

    @Override
    public void setRemoverRollerSpeed(double speed) {
        rollerMotor.set(speed);
    }

    @Override
    public void updateInputs(RemoverIOInputs inputs) {
        inputs.RemoverRollerVelocity = rollerMotor.getEncoder().getVelocity();
        inputs.RemoverRollerVoltage = rollerMotor.getAppliedOutput();
        inputs.RemoverRollerAmps = rollerMotor.getOutputCurrent();
    }
}
