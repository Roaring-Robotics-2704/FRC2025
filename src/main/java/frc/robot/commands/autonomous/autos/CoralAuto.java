// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.autonomous.autos;

// import com.pathplanner.lib.auto.AutoBuilder;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.RepeatCommand;
// import frc.robot.commands.autonomous.components.GoToReef;

// /* You should consider using the more terse Command factories API instead
// https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class CoralAuto {
//   GoToReef reefOuttake = new GoToReef();
//   /** Creates a new CoralAuto. */
//       public Command getCommand() {
//         return new RepeatCommand(AutoBuilder.andThen(sourceIntake)).repeatedly().withTimeout(15);
//     }
// }
