// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.command_factories.ElevatorFactory;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.subsystems.remover.Remover;

/** Add your docs here. */
public class Autos {
    Drive drive;
    Outtake outtake;
    Elevator elevator;
    Remover remover;
    AutoFactory factory;
    AutoChooser chooser = new AutoChooser();
    Routines routines = new Routines();

    public Autos(Drive drive, Outtake outtake, Elevator elevator, Remover remover) {
        this.drive = drive;
        this.outtake = outtake;
        this.elevator = elevator;
        this.remover = remover;
        this.factory = drive.getFactory();
        initEvents();
        chooser.addRoutine("Two Coral Left", routines::TwoCoralLeftRoutine);
        chooser.addRoutine("Leave", routines::Leave);
        SmartDashboard.putData("Auto chooser", chooser);
    }
    ;

    public Command getAutoCommand() {
        return chooser.selectedCommand();
    }

    public void initEvents() {
        factory.bind("intake", outtake.outtakeInCmd(true));
        factory.bind("outtake", outtake.outtakeOutCmd(true));
        factory.bind("elevatorL4", ElevatorFactory.elevatorL4(elevator));
        factory.bind("elevatorL3", ElevatorFactory.elevatorL3(elevator));
        factory.bind("elevatorL2", ElevatorFactory.elevatorL2(elevator));
        factory.bind("elevatorL1", ElevatorFactory.elevatorL1(elevator));
        factory.bind("elevatorIntake", ElevatorFactory.elevatorIntake(elevator));
        factory.bind("armOut", remover.ArmOutAuto());
        factory.bind("armIn", remover.ArmInAuto());
    }

    private class Routines {

        private AutoRoutine TwoCoralLeftRoutine() {
            AutoRoutine routine = factory.newRoutine("TwoCoralLeft");

            // Load the routine's trajectories
            AutoTrajectory twoCoralLeftTraj = routine.trajectory("2 coral left");

            // When the routine begins, reset odometry and start the first trajectory (1)
            routine.active().onTrue(Commands.sequence(twoCoralLeftTraj.resetOdometry(), twoCoralLeftTraj.cmd()));

            return routine;
        }

        private AutoRoutine Leave() {
            AutoRoutine routine = factory.newRoutine("Leave");

            // Load the routine's trajectories
            AutoTrajectory leavTeaj = routine.trajectory("Leave");

            // When the routine begins, reset odometry and start the first trajectory (1)
            routine.active().onTrue(Commands.sequence(leavTeaj.resetOdometry(), leavTeaj.cmd()));

            return routine;
        }
    }
}
