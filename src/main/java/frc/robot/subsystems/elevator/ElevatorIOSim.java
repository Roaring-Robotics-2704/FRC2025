package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.elevator.ElevatorConstants.CARRIAGE_MASS;
import static frc.robot.subsystems.elevator.ElevatorConstants.ELEVATOR_GEARBOX;
import static frc.robot.subsystems.elevator.ElevatorConstants.ELEVATOR_KD;
import static frc.robot.subsystems.elevator.ElevatorConstants.ELEVATOR_KI;
import static frc.robot.subsystems.elevator.ElevatorConstants.ELEVATOR_KP;
import static frc.robot.subsystems.elevator.ElevatorConstants.GEAR_REDUCTION;
import static frc.robot.subsystems.elevator.ElevatorConstants.MAX_HEIGHT;
import static frc.robot.subsystems.elevator.ElevatorConstants.MIN_HEIGHT;
import static frc.robot.subsystems.elevator.ElevatorConstants.PULLEY_RADIUS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
    private final ElevatorSim sim = new ElevatorSim(
            ELEVATOR_GEARBOX,
            GEAR_REDUCTION,
            CARRIAGE_MASS,
            PULLEY_RADIUS,
            Inches.of(MIN_HEIGHT).in(Meters),
            Inches.of(MAX_HEIGHT).in(Meters),
            true,
            Inches.of(MIN_HEIGHT).in(Meters));

    private final PIDController controller = new PIDController(ELEVATOR_KP, ELEVATOR_KI, ELEVATOR_KD);
    private final MutVoltage appliedVolts = Volts.mutable(0);

    @Override
    public void runVolts(Voltage volts) {
        double clampedEffort = MathUtil.clamp(volts.in(Volts), -12, 12);
        appliedVolts.mut_replace(clampedEffort, Volts);
        sim.setInputVoltage(clampedEffort);
    }

    @Override
    public void runSetpoint(Distance position) {
        Distance currentHeight = Meters.of(sim.getPositionMeters());

        Voltage controllerVoltage = Volts.of(controller.calculate(currentHeight.in(Inches), position.in(Inches)));

        runVolts(controllerVoltage);
    }

    @Override
    public void stop() {
        runVolts(Volts.zero());
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        sim.update(0.02);
        inputs.position.mut_replace(sim.getPositionMeters(), Meters);
        inputs.velocity.mut_replace(sim.getVelocityMetersPerSecond(), MetersPerSecond);
        inputs.appliedVolts.mut_replace(appliedVolts);
        inputs.supplyCurrent.mut_replace(sim.getCurrentDrawAmps(), Amps);
        inputs.torqueCurrent.mut_replace(sim.getCurrentDrawAmps(), Amps);
    }

    @Override
    public void init() {}
}
