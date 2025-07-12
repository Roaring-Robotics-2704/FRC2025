// package frc.robot.subsystems.vision;

// import static org.junit.jupiter.api.Assertions.*;
// import static org.mockito.Mockito.*;

// import edu.wpi.first.math.geometry.Rotation2d;
// import org.junit.jupiter.api.Test;

// public class VisionTest {
//     @Test
//     void testGetTargetX() {
//         VisionIO mockIO = mock(VisionIO.class);
//         VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
//         inputs.latestTargetObservation = new VisionIO.TargetObservation(new Rotation2d(1.0), 0.0);
//         when(mockIO.getInputs()).thenReturn(inputs);

//         Vision vision = new Vision((pose, timestamp, stdDevs) -> {}, mockIO);
//         assertEquals(1.0, vision.getTargetX(0).getRadians(), 1e-5);
//     }

//     @Test
//     void testPeriodic() {
//         VisionIO mockIO = mock(VisionIO.class);
//         VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
//         inputs.connected = true;
//         when(mockIO.getInputs()).thenReturn(inputs);

//         Vision vision = new Vision((pose, timestamp, stdDevs) -> {}, mockIO);
//         vision.periodic();

//         verify(mockIO, times(1)).updateInputs(any());
//     }
// }
