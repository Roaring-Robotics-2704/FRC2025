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
    }

    @Override
    public void fromLog(LogTable table) {
        RemoverRollerVelocity = table.get("RemoverRollerVelocity", RemoverRollerVelocity);
        RemoverRollerAmps = table.get("RemoverRollerAmps", RemoverRollerAmps);
        RemoverRollerVoltage = table.get("RemoverRollerVoltage", RemoverRollerVoltage);
        RemoverRollerSpeed = table.get("RemoverRollerSpeed", RemoverRollerSpeed);
    }

    public RemoverIOInputsAutoLogged clone() {
        RemoverIOInputsAutoLogged copy = new RemoverIOInputsAutoLogged();
        copy.RemoverRollerVelocity = this.RemoverRollerVelocity;
        copy.RemoverRollerAmps = this.RemoverRollerAmps;
        copy.RemoverRollerVoltage = this.RemoverRollerVoltage;
        copy.RemoverRollerSpeed = this.RemoverRollerSpeed;
        return copy;
    }
}
