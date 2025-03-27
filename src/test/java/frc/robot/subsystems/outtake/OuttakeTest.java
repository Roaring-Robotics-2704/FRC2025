package frc.robot.subsystems.outtake;

import static org.mockito.Mockito.*;

import org.junit.jupiter.api.Test;

class OuttakeTest {
    @Test
    void testOuttakeOutCmd() {
        OuttakeIO mockIO = mock(OuttakeIO.class);
        Outtake outtake = new Outtake(mockIO, null);

        outtake.outtakeOutCmd(false).initialize();
        verify(mockIO, times(1)).setSpeed(OuttakeConstants.OUTTAKE_SPEED);
    }

    @Test
    void testOuttakeInCmd() {
        OuttakeIO mockIO = mock(OuttakeIO.class);
        Outtake outtake = new Outtake(mockIO, null);

        outtake.outtakeInCmd(false).initialize();
        verify(mockIO, times(1)).setSpeed(OuttakeConstants.INTAKE_SPEED);
    }
}
