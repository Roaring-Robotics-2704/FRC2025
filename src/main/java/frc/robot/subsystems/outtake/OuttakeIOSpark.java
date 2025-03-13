package frc.robot.subsystems.outtake;

import static frc.robot.subsystems.elevator.ElevatorConstants.CURRENT_LIMIT;
import static frc.robot.subsystems.outtake.OuttakeConstants.OUTTAKE_ID;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkMaxConfig;

// Defines the Outtake Class for spark motors and its prtoperties

public class OuttakeIOSpark implements OuttakeIO {
    private SparkMax motor;

    public OuttakeIOSpark() {
        motor = new SparkMax(OUTTAKE_ID, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();

        config.smartCurrentLimit(CURRENT_LIMIT).voltageCompensation(12);
        config.limitSwitch.forwardLimitSwitchEnabled(false);
        config.limitSwitch.reverseLimitSwitchEnabled(false);
        config.limitSwitch.forwardLimitSwitchType(Type.kNormallyOpen);
        config.limitSwitch.setSparkMaxDataPortConfig();

        tryUntilOk(
                motor,
                5,
                () -> motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }

    @Override
    public void setSpeed(double speed) {
        motor.set(speed);
    }

    @Override
    public void updateInputs(OuttakeIOInputs inputs) {
        inputs.connected = !motor.getFaults().can;
        inputs.velocityRadPerSec = motor.getEncoder().getVelocity();
        inputs.voltage = motor.getBusVoltage();
        inputs.currentAmps = motor.getOutputCurrent();
        inputs.outtakeLoaded = motor.getForwardLimitSwitch().isPressed();
    }
}
