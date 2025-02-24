package frc.robot.subsystems.remover;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Remover extends SubsystemBase {
    private RemoverIO removerIO;

    private RemoverIOInputsAutoLogged removerIOInputsAutoLogged = new RemoverIOInputsAutoLogged();

    public Remover(RemoverIO removerIO) {
        this.removerIO = removerIO;
    }

    @Override
    public void periodic() {
        removerIO.updateInputs(removerIOInputsAutoLogged);
    }

    Command ArmOut() {
        return new RunCommand(() -> removerIO.setRemoverRollerSpeed(RemoverConstants.ROLLER_SPEED))
                .repeatedly()
                .withTimeout(1);
    }
    Command ArmIn() {
        return new RunCommand(() -> removerIO.setRemoverRollerSpeed(-RemoverConstants.ROLLER_SPEED))
                .repeatedly()
                .withTimeout(1);
    }
}
