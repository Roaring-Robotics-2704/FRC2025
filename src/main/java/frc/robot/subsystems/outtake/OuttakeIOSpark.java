package frc.robot.subsystems.outtake;

import static frc.robot.subsystems.outtake.OuttakeConstants.OUTTAKE_ID;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.subsystems.outtake.OuttakeConstants.*;
import frc.robot.subsystems.outtake.OuttakeIO.OuttakeIOInputs;

public class OuttakeIOSpark implements OuttakeIO {
    private SparkMax motor;

    public OuttakeIOSpark() {
        motor = new SparkMax(OUTTAKE_ID, MotorType.kBrushless);
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
    }
    ;
}
