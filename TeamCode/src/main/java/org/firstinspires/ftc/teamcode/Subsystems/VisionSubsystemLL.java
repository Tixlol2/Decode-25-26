package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.Subsystems.Slots.MainSlot;

import java.util.ArrayList;
import java.util.List;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

@Configurable
public class VisionSubsystemLL implements Subsystem {
    public static final VisionSubsystemLL INSTANCE = new VisionSubsystemLL();

    private Limelight3A limelight;
    private LLResult latestResult;

    // Cached detection lists, refreshed each periodic()
    private ArrayList<Integer> detectionIDs = new ArrayList<>();
    private ArrayList<LLResultTypes.FiducialResult> fiducials = new ArrayList<>();
    private ArrayList<MainSlot.SlotState> pattern = new ArrayList<>();

    public VisionSubsystemLL() {}

    @Override
    public void initialize() {
        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0); // AprilTag pipeline — configure in the Limelight web UI
        limelight.start();
    }

    /**
     * Polls the Limelight for the latest result and refreshes detection caches.
     * Called automatically every loop via periodic().
     */
    public void getDetections() {
        latestResult = limelight.getLatestResult();

        fiducials.clear();
        detectionIDs.clear();

        if (latestResult != null && latestResult.isValid()) {
            List<LLResultTypes.FiducialResult> rawFiducials = latestResult.getFiducialResults();
            if (rawFiducials != null) {
                for (LLResultTypes.FiducialResult fiducial : rawFiducials) {
                    fiducials.add(fiducial);

                    detectionIDs.add(fiducial.getFiducialId());
                }
            }
        }
    }

    /**
     * Returns the IDs of all currently visible AprilTags.
     */
    public ArrayList<Integer> getDetectionIDs() {
        return detectionIDs;
    }

    /**
     * Returns the Obelisk tag ID (21, 22, or 23) if one is visible, otherwise -1.
     */
    public int getObeliskID() {
        for (int id : detectionIDs) {
            if (id == 21 || id == 22 || id == 23) {
                return id;
            }
        }
        return -1;
    }

    /**
     * Returns the SlotState pattern for the currently visible Obelisk tag.
     */
    public ArrayList<MainSlot.SlotState> getPattern() {
        obeliskTargetPattern(getObeliskID());
        return pattern;
    }

    /**
     * Returns the bearing to the alliance goal tag if visible, otherwise -9999999.
     *
     * Uses getTx() (horizontal offset in degrees from crosshair) as the bearing equivalent.
     * This matches the original VisionPortal ftcPose.bearing behavior for targeting purposes.
     */
    public double getGoalBearing() {
        if (!isGoalVisible()) {
            return -9999999;
        }

        int goalID = RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.BLUE ? 20 : 24;

        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            if (fiducial.getFiducialId() == goalID) {
                // tx is the horizontal angular offset from the crosshair (degrees)
                return fiducial.getTargetXDegrees();
            }
        }

        return -9999999;
    }

    /**
     * Returns true if the alliance-appropriate goal tag is currently visible.
     */
    public boolean isGoalVisible() {
        boolean isBlue = RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.BLUE;
        return isBlue ? detectionIDs.contains(20) : detectionIDs.contains(24);
    }

    /**
     * Stops the Limelight.
     */
    public void stop() {
        limelight.stop();
    }

    public void resetPattern() {
        pattern = new ArrayList<>(java.util.Arrays.asList(null, null, null));
    }

    public void obeliskTargetPattern(int ID) {
        switch (ID) {
            case -1:
                pattern = new ArrayList<>(java.util.Arrays.asList(null, null, null));
                break;
            case 21:
                pattern = new ArrayList<>(java.util.Arrays.asList(
                        MainSlot.SlotState.GREEN, MainSlot.SlotState.PURPLE, MainSlot.SlotState.PURPLE));
                break;
            case 22:
                pattern = new ArrayList<>(java.util.Arrays.asList(
                        MainSlot.SlotState.PURPLE, MainSlot.SlotState.GREEN, MainSlot.SlotState.PURPLE));
                break;
            case 23:
                pattern = new ArrayList<>(java.util.Arrays.asList(
                        MainSlot.SlotState.PURPLE, MainSlot.SlotState.PURPLE, MainSlot.SlotState.GREEN));
                break;
        }
    }

    @Override
    public void periodic() {
        getDetections();
    }
}