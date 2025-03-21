package frc.robot.subsystems.elevator;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ElevatorIOInputsAutoLogged extends ElevatorIO.ElevatorIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("ElevatorConnected", elevatorConnected);
    table.put("ElevatorHeight", elevatorHeight);
    table.put("ElevatorVelocity", elevatorVelocity);
    table.put("RightElevatorAppliedVolts", rightElevatorAppliedVolts);
    table.put("LeftElevatorAppliedVolts", leftElevatorAppliedVolts);
    table.put("RightElevatorCurrentAmps", rightElevatorCurrentAmps);
    table.put("LeftElevatorCurrentAmps", leftElevatorCurrentAmps);
  }

  @Override
  public void fromLog(LogTable table) {
    elevatorConnected = table.get("ElevatorConnected", elevatorConnected);
    elevatorHeight = table.get("ElevatorHeight", elevatorHeight);
    elevatorVelocity = table.get("ElevatorVelocity", elevatorVelocity);
    rightElevatorAppliedVolts = table.get("RightElevatorAppliedVolts", rightElevatorAppliedVolts);
    leftElevatorAppliedVolts = table.get("LeftElevatorAppliedVolts", leftElevatorAppliedVolts);
    rightElevatorCurrentAmps = table.get("RightElevatorCurrentAmps", rightElevatorCurrentAmps);
    leftElevatorCurrentAmps = table.get("LeftElevatorCurrentAmps", leftElevatorCurrentAmps);
  }

  public ElevatorIOInputsAutoLogged clone() {
    ElevatorIOInputsAutoLogged copy = new ElevatorIOInputsAutoLogged();
    copy.elevatorConnected = this.elevatorConnected;
    copy.elevatorHeight = this.elevatorHeight;
    copy.elevatorVelocity = this.elevatorVelocity;
    copy.rightElevatorAppliedVolts = this.rightElevatorAppliedVolts;
    copy.leftElevatorAppliedVolts = this.leftElevatorAppliedVolts;
    copy.rightElevatorCurrentAmps = this.rightElevatorCurrentAmps;
    copy.leftElevatorCurrentAmps = this.leftElevatorCurrentAmps;
    return copy;
  }
}
