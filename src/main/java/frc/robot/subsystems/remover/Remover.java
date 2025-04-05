package frc.robot.subsystems.remover;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.command_factories.ElevatorFactory;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;

public class Remover extends SubsystemBase {
    private RemoverIO removerIO;

    // private RemoverIOInputsAutoLogged removerIOInputsAutoLogged = new RemoverIOInputsAutoLogged();

    public Remover(RemoverIO removerIO) {
        this.removerIO = removerIO;
    }

    @Override
    public void periodic() {
        // removerIO.updateInputs(removerIOInputsAutoLogged);
    }

    public Command ArmOut() { // TODO add elevator controls
        return new RunCommand(() -> removerIO.setRemoverRollerSpeed(RemoverConstants.ROLLER_SPEED))
                .finallyDo(() -> removerIO.setRemoverRollerSpeed(0));
    }

    public Command ArmIn() { // TODO add elevator controls
        return new RunCommand(() -> removerIO.setRemoverRollerSpeed(-RemoverConstants.ROLLER_SPEED))
                .finallyDo(() -> removerIO.setRemoverRollerSpeed(0));
    }

    public Command ArmOutAuto() {
        return new RunCommand(() -> removerIO.setRemoverRollerSpeed(RemoverConstants.ROLLER_SPEED))
                .repeatedly()
                .withTimeout(0.5)
                .andThen(Commands.runOnce(() -> removerIO.setRemoverRollerSpeed(0)))
                .finallyDo(() -> removerIO.setRemoverRollerSpeed(0));
    }

    public Command ArmInAuto() { // TODO add elevator controls
        return new RunCommand(() -> removerIO.setRemoverRollerSpeed(-RemoverConstants.ROLLER_SPEED))
                .repeatedly()
                .withTimeout(0.75)
                .andThen(Commands.runOnce(() -> removerIO.setRemoverRollerSpeed(0)))
                .finallyDo(() -> removerIO.setRemoverRollerSpeed(0));
    }

    public Command L3Algae(Elevator elevator, Drive drive) {
        return Commands.sequence(
                ElevatorFactory.elevatorL4(elevator).withTimeout(0.25),
                ArmOutAuto(),
                ElevatorFactory.ElevatorAlgaeL3(elevator).withTimeout(0.2),
                Commands.run(() -> drive.runVelocity(new ChassisSpeeds(-0.5, 0, 0)), drive)
                        .repeatedly()
                        .withTimeout(0.5),
                Commands.runOnce(() -> drive.runVelocity(new ChassisSpeeds()), drive),
                ArmInAuto());
    }

    public Command L2Algae(Elevator elevator, Drive drive) {
        return Commands.sequence(
                ElevatorFactory.elevatorL3(elevator).withTimeout(0.25),
                ArmOutAuto(),
                ElevatorFactory.ElevatorAlgaeL2(elevator).withTimeout(0.2),
                Commands.run(() -> drive.runVelocity(new ChassisSpeeds(-0.5, 0, 0)), drive)
                        .repeatedly()
                        .withTimeout(0.75),
                Commands.runOnce(() -> drive.runVelocity(new ChassisSpeeds()), drive),
                ArmInAuto());
    }
}
