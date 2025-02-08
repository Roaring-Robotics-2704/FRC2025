package frc.robot.subsystems.outtake;

import static frc.robot.subsystems.outtake.OuttakeConstants.OUTTAKE_ID;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.outtake.OuttakeConstants.*;
import frc.robot.subsystems.outtake.OuttakeIO.OuttakeIOInputs;

// Defines the Outtake Class for spark motors and its prtoperties

public class OuttakeIOSpark implements OuttakeIO {
    private SparkMax motor;
    private DigitalInput outtakeBeambreak;

    public OuttakeIOSpark() {
        motor = new SparkMax(OUTTAKE_ID, MotorType.kBrushless);
        outtakeBeambreak = new DigitalInput(21);
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
