package frc.robot.subsystems.remover;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Remover extends SubsystemBase {
    private RemoverIO removerIO;

    public Remover(RemoverIO removerIO) {
        this.removerIO = removerIO;
    }

    @Override
    public void periodic() {}

    Command pivotCommand() {
        int pivotChecker = 0;
        pivotChecker++;

        if ((pivotChecker % 2) == 0) {
            return new RunCommand(() -> removerIO.setRemoverRollerPositionRad(RemoverConstants.PIVOT_SPEED))
                    .repeatedly()
                    .withTimeout(1);
        } else {
            return new RunCommand(() -> removerIO.setRemoverRollerPositionRad(RemoverConstants.PIVOT_SPEED * -1))
                    .repeatedly()
                    .withTimeout(1);
        }
    }

    Command rollerCommand() {
        return new RunCommand(() -> removerIO.setRemoverRollerSpeed(RemoverConstants.ROLLER_SPEED))
                .repeatedly()
                .withTimeout(1);
    }
}
