package frc.robot.subsystems.algaeArm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.robot.subsystems.algaeArm.AlgaeArmConstants.*;
import frc.robot.subsystems.algaeArm.AlgaeArmIO.AlgaeArmIOInputs;
import frc.robot.util.SparkUtil.*;

public class AlgaeArmIOSim implements AlgaeArmIO {
    private double m_armKp = 3;
    private double m_armSetpointDegrees = 0;

    // The arm gearbox represents a gearbox containing two Vex 775pro motors.
    private final DCMotor m_armGearbox = DCMotor.getVex775Pro(2);

    // Standard classes for controlling our arm
    private final PIDController m_controller = new PIDController(m_armKp, 0, 0);
    private final Encoder m_encoder = new Encoder(1, 2);
    private final PWMSparkMax m_motor = new PWMSparkMax(28);

    // Simulation classes help us simulate what's going on, including gravity.
    // This arm sim represents an arm that can travel from -75 degrees (rotated down front)
    // to 255 degrees (rotated down in the back).
    private final SingleJointedArmSim m_armSim = new SingleJointedArmSim(
            m_armGearbox,
            1,
            SingleJointedArmSim.estimateMOI(Units.inchesToMeters(24), Units.lbsToKilograms(15)),
            Units.inchesToMeters(24),
            AlgaeArmConstants.INSIDE_POSITION,
            AlgaeArmConstants.INTAKE_POSITION,
            true,
            AlgaeArmConstants.INSIDE_POSITION,
            4096,
            0.0 // Add noise with a std-dev of 1 tick
            );
    private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);

    // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
    private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
    private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 30, 30);
    private final MechanismLigament2d m_armTower = m_armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
    private final MechanismLigament2d m_arm =
            m_armPivot.append(new MechanismLigament2d("Arm", 30, Units.radiansToDegrees(m_armSim.getAngleRads())));

    public AlgaeArmIOSim() {}

    @Override
    public void setAlgaeArmPosition(double radians) {
        m_armSim.setInput(m_controller.calculate(m_armSim.getAngleRads(), radians));
    }

    @Override
    public void setRollerSpeed(double speed) {}

    @Override
    public void updateInputs(AlgaeArmIOInputs inputs) {
        m_armSim.update(0.02);
        inputs.algaePivotPositionRad = m_armSim.getAngleRads();
        inputs.algaePivotVelocity = m_encoderSim.getRate();
        inputs.algaeRollerVelocity = 0;
        inputs.algaePivotAppliedVolts = 0;
        inputs.algaeRollerVoltage = 0;
        inputs.algaePivotAmps = 0;
        inputs.algaeRollerAmps = 0;
    }
}
