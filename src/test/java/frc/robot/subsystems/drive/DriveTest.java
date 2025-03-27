package frc.robot.subsystems.drive;

import static org.mockito.Mockito.*;

import edu.wpi.first.math.geometry.Pose2d;
import org.junit.jupiter.api.Test;

public class DriveTest {
    @Test
    void testRunVelocity() {
        ModuleIO mockModule = mock(ModuleIO.class);
        Drive drive = new Drive(null, mockModule, mockModule, mockModule, mockModule);

        drive.runVelocity(null);
        verify(mockModule, times(4)).setDriveVelocity(anyDouble());
    }

    @Test
    void testResetOdometry() {
        ModuleIO mockModule = mock(ModuleIO.class);
        Drive drive = new Drive(null, mockModule, mockModule, mockModule, mockModule);

        drive.resetOdometry(new Pose2d());
        verify(mockModule, times(4)).updateInputs(any());
    }
}
