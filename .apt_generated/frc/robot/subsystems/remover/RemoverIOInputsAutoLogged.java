package frc.robot.subsystems.remover;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class RemoverIOInputsAutoLogged extends RemoverIO.RemoverIOInputs implements LoggableInputs, Cloneable {
    @Override
    public void toLog(LogTable table) {
        table.put("RemoverRollerVelocity", RemoverRollerVelocity);
        table.put("RemoverRollerAmps", RemoverRollerAmps);
        table.put("RemoverRollerVoltage", RemoverRollerVoltage);
        table.put("RemoverRollerSpeed", RemoverRollerSpeed);
        table.put("RemoverPivotPositionRad", RemoverPivotPositionRad);
        table.put("RemoverPivotAppliedVolts", RemoverPivotAppliedVolts);
        table.put("RemoverPivotVelocity", RemoverPivotVelocity);
        table.put("RemoverPivotAmps", RemoverPivotAmps);
    }

    @Override
    public void fromLog(LogTable table) {
        RemoverRollerVelocity = table.get("RemoverRollerVelocity", RemoverRollerVelocity);
        RemoverRollerAmps = table.get("RemoverRollerAmps", RemoverRollerAmps);
        RemoverRollerVoltage = table.get("RemoverRollerVoltage", RemoverRollerVoltage);
        RemoverRollerSpeed = table.get("RemoverRollerSpeed", RemoverRollerSpeed);
        RemoverPivotPositionRad = table.get("RemoverPivotPositionRad", RemoverPivotPositionRad);
        RemoverPivotAppliedVolts = table.get("RemoverPivotAppliedVolts", RemoverPivotAppliedVolts);
        RemoverPivotVelocity = table.get("RemoverPivotVelocity", RemoverPivotVelocity);
        RemoverPivotAmps = table.get("RemoverPivotAmps", RemoverPivotAmps);
    }

    public RemoverIOInputsAutoLogged clone() {
        RemoverIOInputsAutoLogged copy = new RemoverIOInputsAutoLogged();
        copy.RemoverRollerVelocity = this.RemoverRollerVelocity;
        copy.RemoverRollerAmps = this.RemoverRollerAmps;
        copy.RemoverRollerVoltage = this.RemoverRollerVoltage;
        copy.RemoverRollerSpeed = this.RemoverRollerSpeed;
        copy.RemoverPivotPositionRad = this.RemoverPivotPositionRad;
        copy.RemoverPivotAppliedVolts = this.RemoverPivotAppliedVolts;
        copy.RemoverPivotVelocity = this.RemoverPivotVelocity;
        copy.RemoverPivotAmps = this.RemoverPivotAmps;
        return copy;
    }
}
