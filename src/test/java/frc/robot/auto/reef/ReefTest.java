package frc.robot.auto.reef;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import org.junit.jupiter.api.Test;

public class ReefTest {
    @Test
    void testGetClosestBranch() {
        Reef reef = new Reef();
        Pose2d pose = new Pose2d();

        Branch branch = reef.getclosestBranch(pose, Branch.Level.L3, false);
        assertNotNull(branch);
    }

    @Test
    void testIsReefFull() {
        Reef reef = new Reef();
        assertFalse(reef.isReefFull());
    }
}
