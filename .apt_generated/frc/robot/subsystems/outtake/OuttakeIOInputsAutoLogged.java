package frc.robot.subsystems.outtake;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class OuttakeIOInputsAutoLogged extends OuttakeIO.OuttakeIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Connected", connected);
    table.put("VelocityRadPerSec", velocityRadPerSec);
    table.put("Voltage", voltage);
    table.put("CurrentAmps", currentAmps);
    table.put("OuttakeLoaded", outtakeLoaded);
  }

  @Override
  public void fromLog(LogTable table) {
    connected = table.get("Connected", connected);
    velocityRadPerSec = table.get("VelocityRadPerSec", velocityRadPerSec);
    voltage = table.get("Voltage", voltage);
    currentAmps = table.get("CurrentAmps", currentAmps);
    outtakeLoaded = table.get("OuttakeLoaded", outtakeLoaded);
  }

  public OuttakeIOInputsAutoLogged clone() {
    OuttakeIOInputsAutoLogged copy = new OuttakeIOInputsAutoLogged();
    copy.connected = this.connected;
    copy.velocityRadPerSec = this.velocityRadPerSec;
    copy.voltage = this.voltage;
    copy.currentAmps = this.currentAmps;
    copy.outtakeLoaded = this.outtakeLoaded;
    return copy;
  }
}
