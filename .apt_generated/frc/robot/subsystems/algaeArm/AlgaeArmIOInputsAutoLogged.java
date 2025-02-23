package frc.robot.subsystems.algaeArm;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class AlgaeArmIOInputsAutoLogged extends AlgaeArmIO.AlgaeArmIOInputs implements LoggableInputs, Cloneable {
    @Override
    public void toLog(LogTable table) {
        table.put("AlgaePivotPositionRad", algaePivotPositionRad);
        table.put("AlgaePivotVelocity", algaePivotVelocity);
        table.put("AlgaePivotAppliedVolts", algaePivotAppliedVolts);
        table.put("AlgaePivotAmps", algaePivotAmps);
        table.put("AlgaeRollerVelocity", algaeRollerVelocity);
        table.put("AlgaeRollerAmps", algaeRollerAmps);
        table.put("AlgaeRollerVoltage", algaeRollerVoltage);
        table.put("AlgaeRollerSpeed", algaeRollerSpeed);
    }

    @Override
    public void fromLog(LogTable table) {
        algaePivotPositionRad = table.get("AlgaePivotPositionRad", algaePivotPositionRad);
        algaePivotVelocity = table.get("AlgaePivotVelocity", algaePivotVelocity);
        algaePivotAppliedVolts = table.get("AlgaePivotAppliedVolts", algaePivotAppliedVolts);
        algaePivotAmps = table.get("AlgaePivotAmps", algaePivotAmps);
        algaeRollerVelocity = table.get("AlgaeRollerVelocity", algaeRollerVelocity);
        algaeRollerAmps = table.get("AlgaeRollerAmps", algaeRollerAmps);
        algaeRollerVoltage = table.get("AlgaeRollerVoltage", algaeRollerVoltage);
        algaeRollerSpeed = table.get("AlgaeRollerSpeed", algaeRollerSpeed);
    }

    public AlgaeArmIOInputsAutoLogged clone() {
        AlgaeArmIOInputsAutoLogged copy = new AlgaeArmIOInputsAutoLogged();
        copy.algaePivotPositionRad = this.algaePivotPositionRad;
        copy.algaePivotVelocity = this.algaePivotVelocity;
        copy.algaePivotAppliedVolts = this.algaePivotAppliedVolts;
        copy.algaePivotAmps = this.algaePivotAmps;
        copy.algaeRollerVelocity = this.algaeRollerVelocity;
        copy.algaeRollerAmps = this.algaeRollerAmps;
        copy.algaeRollerVoltage = this.algaeRollerVoltage;
        copy.algaeRollerSpeed = this.algaeRollerSpeed;
        return copy;
    }
}
