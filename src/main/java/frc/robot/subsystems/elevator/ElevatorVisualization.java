package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

public class ElevatorVisualization {
    // === CONFIGURABLE CONSTANTS ===
    private static final int STAGE_COUNT = 3; // Number of moving stages
    private static final double STAGE_HEIGHT = 26; // Height of each stage
    private static final double CARRIAGE_HEIGHT = 6; // Height of the carriage
    private static final double OVERLAP = 6; // Overlap between stages
    private static final double FRAME_CLEARANCE = 1; // Stops 1 inch before full extension
    private static final double MECH_WIDTH = 150; // Mechanism2D display width
    private static final double MECH_HEIGHT = 100; // Mechanism2D display height

    private final Mechanism2d mech;
    private final MechanismLigament2d[] stages;
    private final MechanismLigament2d carriage;

    public ElevatorVisualization() {
        mech = new Mechanism2d(MECH_WIDTH, MECH_HEIGHT); // Set visualization size
        MechanismRoot2d base = mech.getRoot("Base", MECH_WIDTH / 2, 10); // Base at y=10

        // Create stage array
        stages = new MechanismLigament2d[STAGE_COUNT];

        // Initialize first stage from base
        stages[0] = base.append(new MechanismLigament2d("Stage1", 0, 90));

        // Create remaining stages, attaching each to the previous one
        for (int i = 1; i < STAGE_COUNT; i++) {
            stages[i] = stages[i - 1].append(new MechanismLigament2d("Stage" + (i + 1), 0, 90));
        }

        // Carriage attaches to the last stage
        carriage = stages[STAGE_COUNT - 1].append(new MechanismLigament2d("Carriage", 0, 90));

        // Add to Shuffleboard for display
        ShuffleboardTab tab = Shuffleboard.getTab("Elevator");
        tab.add("Elevator Mechanism", mech)
                .withSize(6, 6) // Make it bigger in Shuffleboard
                .withPosition(0, 0);
    }

    public void update(double totalHeight) {
        // Constrain total height to account for overlap and framing
        double maxHeight = (STAGE_COUNT * STAGE_HEIGHT) - ((STAGE_COUNT - 1) * OVERLAP) - FRAME_CLEARANCE;
        totalHeight = Math.max(0, Math.min(totalHeight, maxHeight));

        // Calculate stage extension with overlap correction
        double stageExtension = (totalHeight + (STAGE_COUNT - 1) * OVERLAP) / STAGE_COUNT;

        // Apply extension to all stages
        for (MechanismLigament2d stage : stages) {
            stage.setLength(stageExtension);
        }

        // Carriage stays inside last stage
        carriage.setLength(CARRIAGE_HEIGHT);
    }

    public Mechanism2d getMechanism() {
        return mech;
    }
}
