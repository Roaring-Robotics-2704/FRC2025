package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Encoder;
import static frc.robot.subsystems.elevator.ElevatorConstants.CURRENT_LIMIT;
import static frc.robot.subsystems.elevator.ElevatorConstants.DISTANCE_PER_PULSE;
import static frc.robot.subsystems.elevator.ElevatorConstants.ELEVATOR_KD;
import static frc.robot.subsystems.elevator.ElevatorConstants.ELEVATOR_KI;
import static frc.robot.subsystems.elevator.ElevatorConstants.ELEVATOR_KP;
import static frc.robot.subsystems.elevator.ElevatorConstants.ENCODER_A;
import static frc.robot.subsystems.elevator.ElevatorConstants.ENCODER_B;
import static frc.robot.util.SparkUtil.tryUntilOk;

public class ElevatorIOSpark implements ElevatorIO {

    public static final SparkMax motor1 =
            new SparkMax(ElevatorConstants.ELEVATOR_MOTOR_1, SparkMax.MotorType.kBrushless);
    public static final SparkMax motor2 =
            new SparkMax(ElevatorConstants.ELEVATOR_MOTOR_2, SparkMax.MotorType.kBrushless);

    private static final Encoder m_encoder = new Encoder(ENCODER_A, ENCODER_B);
    public static final SparkMaxConfig motorConfig = new SparkMaxConfig();

    private final PIDController controller = new PIDController(ELEVATOR_KP, ELEVATOR_KI, ELEVATOR_KD);

    @Override
    public void init() {
        m_encoder.setDistancePerPulse(DISTANCE_PER_PULSE);

        motorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(CURRENT_LIMIT).voltageCompensation(12.0);
        motorConfig.inverted(false);
        tryUntilOk(
                motor1,
                5,
                () -> motor1.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        tryUntilOk(
                motor2,
                5,
                () -> motor2.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.position.mut_replace(m_encoder.getDistance(), Meters);        
        inputs.velocity.mut_replace(m_encoder.getRate(), MetersPerSecond);
        inputs.appliedVolts.mut_replace(motor1.getAppliedOutput() * 12.0, Volts);
        inputs.supplyCurrent.mut_replace(motor1.getOutputCurrent(), Amps);
        inputs.torqueCurrent.mut_replace(motor1.getOutputCurrent(), Amps);
    }    @Override
    public void runVolts(Voltage volts) {
    double clampedEffort = MathUtil.clamp(volts.in(Volts), -12, 12);
      motor1.setVoltage(clampedEffort);
      motor2.setVoltage(clampedEffort);
    }

    @Override
    public void runSetpoint(Distance position) {
        Distance currentHeight = Meters.of(m_encoder.getDistance());

        Voltage controllerVoltage = Volts.of(controller.calculate(currentHeight.in(Inches), position.in(Inches)));

        runVolts(controllerVoltage);
    }

    @Override
    public void stop() {
        runVolts(Volts.zero());
    }
}
