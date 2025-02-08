package frc.robot.commands.Autonomus.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Locations.ReefChooser;
import frc.robot.commands.DriveCommands;
import java.util.function.Supplier;

public class DriveToReef {
    public static Supplier<Command> DrivetoReef(ReefChooser chooser) {
        return () -> DriveCommands.pathfindPose(chooser::getReefPose);
    }
}
