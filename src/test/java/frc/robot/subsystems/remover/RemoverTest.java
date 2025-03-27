package frc.robot.subsystems.remover;

import static org.mockito.Mockito.*;

import org.junit.jupiter.api.Test;

class RemoverTest {
    @Test
    void testArmOut() {
        RemoverIO mockIO = mock(RemoverIO.class);
        Remover remover = new Remover(mockIO);

        remover.ArmOut().initialize();
        verify(mockIO, times(1)).setRemoverRollerSpeed(RemoverConstants.ROLLER_SPEED);
    }

    @Test
    void testArmIn() {
        RemoverIO mockIO = mock(RemoverIO.class);
        Remover remover = new Remover(mockIO);

        remover.ArmIn().initialize();
        verify(mockIO, times(1)).setRemoverRollerSpeed(-RemoverConstants.ROLLER_SPEED);
    }
}
