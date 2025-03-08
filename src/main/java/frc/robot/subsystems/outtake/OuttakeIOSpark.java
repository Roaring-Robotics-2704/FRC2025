package frc.robot.subsystems.outtake;

import static frc.robot.subsystems.elevator.ElevatorConstants.CURRENT_LIMIT;
import static frc.robot.subsystems.outtake.OuttakeConstants.OUTTAKE_ID;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;

// Defines the Outtake Class for spark motors and its prtoperties

public class OuttakeIOSpark implements OuttakeIO {
    private SparkMax motor;
    private DigitalInput outtakeBeambreak;

    public OuttakeIOSpark() {
        motor = new SparkMax(OUTTAKE_ID, MotorType.kBrushless);
        outtakeBeambreak = new DigitalInput(OuttakeConstants.OUTTAKE_BEAMBREAK_ID);
        SparkMaxConfig config = new SparkMaxConfig();

        config.smartCurrentLimit(CURRENT_LIMIT).voltageCompensation(12);

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
        inputs.outtakeLoaded = outtakeBeambreak.get();
    }
}
