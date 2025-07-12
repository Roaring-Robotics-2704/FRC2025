// package frc.robot.subsystems.elevator;

// import static org.mockito.Mockito.*;

// import org.junit.jupiter.api.Test;

// class ElevatorTest {
//     @Test
//     void testSetElevatorHeight() {
//         ElevatorIO mockIO = mock(ElevatorIO.class);
//         Elevator elevator = new Elevator(mockIO, null);

//         elevator.setElevatorHeight(2.0);
//         verify(mockIO, times(1)).runVolts(any());
//     }

//     @Test
//     void testGetHeight() {
//         ElevatorIO mockIO = mock(ElevatorIO.class);
//         ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
//         inputs.elevatorHeight = 1.5;
//         when(mockIO.getInputs()).thenReturn(inputs);

//         Elevator elevator = new Elevator(mockIO, null);
//         assertEquals(1.5, elevator.getHeight(), 1e-5);
//     }
// }
