package frc.robot.subsystems.remover;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static frc.robot.subsystems.elevator.ElevatorConstants.CURRENT_LIMIT;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.util.SparkUtil;

public class RemoverIOSpark implements RemoverIO {
    private SparkMax rollerMotor;

    public RemoverIOSpark() {
        rollerMotor = new SparkMax(RemoverConstants.REMOVER_CANID, MotorType.kBrushless);
                SparkMaxConfig config = new SparkMaxConfig();

        config.smartCurrentLimit(CURRENT_LIMIT).voltageCompensation(12);
        
        SparkUtil.tryUntilOk(rollerMotor, 5, ()->rollerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
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
