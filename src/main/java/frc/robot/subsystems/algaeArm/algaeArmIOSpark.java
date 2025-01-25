package frc.robot.subsystems.algaeArm;

import frc.robot.subsystems.algaeArm.algaeArmConstants;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;

import edu.wpi.first.wpilibj.Encoder;



public class algaeArmIOSpark implements algaeArmIO {
	private SparkMax rollerMotor;
	private SparkMax pivotMotor;
	private AbsoluteEncoder throughBore;


	public algaeArmIOSpark() {

		rollerMotor= new SparkMax(algaeArmConstants.ROLLER_MOTOR_CANID, MotorType.kBrushless);
		pivotMotor= new SparkMax(algaeArmConstants.PIVOT_MOTOR_CANID, MotorType.kBrushless);
		throughBore = pivotMotor.getAbsoluteEncoder();

	}

    
    public void setPivotSpeed(double speed) {
        pivotMotor.set(speed);
	}

	public void setRollerSpeed(double speed) {
        pivotMotor.set(speed);
	}

	public void updateInputs(algaeArmIOInputs inputs) {
		inputs.algaeArmConnected = true;
		inputs.algaeArmPositionRad = throughBore.getPosition();
		inputs.algaeArmVelocity = throughBore.getVelocity();
		inputs.algaeArmAppliedVolts = rollerMotor.getAppliedOutput();
		inputs.algaeArmCurrentAmps = rollerMotor.getOutputCurrent();
	}

}

