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
    private static ArrayList<Integer> detectionIDs = new ArrayList<>();
    private ArrayList<LLResultTypes.FiducialResult> fiducials = new ArrayList<>();
    private ArrayList<MainSlot.SlotState> pattern = new ArrayList<>();

    private int goalID = 0;

    // ── Vision-Turret Correction Filter ──────────────────────────────────────
    //
    // THEORY OF OPERATION:
    //   getGoalBearing() returns raw Limelight tx — the residual angular error
    //   between the camera crosshair and the goal AprilTag.  The LIME turret
    //   state uses this as a fine-tune delta on top of the odometry estimate:
    //
    //       turretTarget = goalAngle_odometry + filteredTx
    //
    //   Raw tx is noisy (±0.5–1° jitter at 200 Hz).  An Exponential Moving
    //   Average smooths it without the phase lag of a boxcar filter.
    //
    //   TX_FILTER_ALPHA  — [0, 1].  Higher = faster response, more noise.
    //                      0.35 is a good starting point; raise if the turret
    //                      feels sluggish, lower if it chatters.
    //
    //   TX_DEADBAND_DEG  — Ignore corrections smaller than this.  Prevents
    //                      the turret from hunting when it is already on-target.
    //                      Set to slightly above your mechanical backlash
    //                      expressed in degrees (typically 0.3–0.6°).
    //
    //   TX_MAX_CORRECTION_DEG — Hard clamp per frame.  Protects against single
    //                           bad detections blasting the turret away.  Set
    //                           to a value larger than your worst-case odometry
    //                           drift (~8–12°) but small enough to reject glitches.
    //
    //   STALE_THRESHOLD_MS — If no fresh goal tag has been seen for this long
    //                        the filtered value is held (not zeroed) and
    //                        isGoalFresh() returns false so the caller can
    //                        fall back to pure odometry.

    /** EMA smoothing coefficient for tx.  Range [0, 1].  @Configurable. */
    public static double TX_FILTER_ALPHA       = 0.35;

    /** Ignore tx corrections smaller than this magnitude (degrees). */
    public static double TX_DEADBAND_DEG       = 0.2;

    /** Max tx correction accepted in a single cycle (degrees). */
    public static double TX_MAX_CORRECTION_DEG = 12.0;

    /** How long (ms) before a goal detection is considered stale. */
    public static long   STALE_THRESHOLD_MS    = 150;

    /** Sign applied to raw tx before filtering.  Flip to -1 if the correction
     *  drives the turret the wrong way after first test. */
    public static double TX_SIGN = 1.0;

    // Filter state
    private double filteredTx       = 0.0;
    private long   lastGoalSeenMs   = 0;
    private boolean goalWasVisible  = false;

    public VisionSubsystemLL() {}

    @Override
    public void initialize() {
        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(200);
        limelight.pipelineSwitch(1); // AprilTag pipeline — configure in the Limelight web UI
        limelight.start();
        goalID = 0;
    }

    /**
     * Polls the Limelight for the latest result and refreshes detection caches.
     * Also updates the tx EMA filter and the goal-freshness timestamp.
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

        // ── Update EMA filter ─────────────────────────────────────────────────
        double rawTx = getRawGoalBearing(); // -9999999 when not visible

        if (rawTx > -999) {
            // Clamp the raw measurement before feeding the filter so a single
            // glitchy frame cannot yank the estimate far from reality.
            double clamped = Math.max(-TX_MAX_CORRECTION_DEG,
                             Math.min( TX_MAX_CORRECTION_DEG, rawTx * TX_SIGN));

            // Exponential moving average
            filteredTx = TX_FILTER_ALPHA * clamped + (1.0 - TX_FILTER_ALPHA) * filteredTx;

            lastGoalSeenMs  = System.currentTimeMillis();
            goalWasVisible  = true;
        } else {
            goalWasVisible = false;
            // Do NOT zero filteredTx here — hold the last known correction so
            // the turret does not snap away when the tag blinks out for one frame.
        }
    }

    // ── Public API ────────────────────────────────────────────────────────────

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
     * Raw (unfiltered) horizontal offset of the goal tag from the camera crosshair.
     * Returns -9999999 when the goal tag is not visible.
     *
     * Prefer getFilteredGoalBearing() for closed-loop turret control.
     */
    public double getRawGoalBearing() {
        if (!isGoalVisible()) return -9999999;

        int targetID = RobotSubsystem.INSTANCE.getAllianceColor()
                == RobotSubsystem.AllianceColor.BLUE ? 20 : 24;

        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            if (fiducial.getFiducialId() == targetID) {
                return fiducial.getTargetXDegrees(); // tx
            }
        }
        return -9999999;
    }

    /**
     * EMA-filtered, deadbanded, clamped angular correction toward the goal tag.
     *
     * Use this as the vision residual in:
     *     turretTarget = goalAngle_odometry + getFilteredGoalBearing()
     *
     * Returns 0.0 when the goal has been stale for longer than STALE_THRESHOLD_MS,
     * so the caller falls back cleanly to pure odometry.
     *
     * @return filtered tx in degrees, or 0.0 if stale
     */
    public double getFilteredGoalBearing() {
        if (!isGoalFresh()) return 0.0;

        // Apply deadband — small residuals below backlash threshold become zero.
        if (Math.abs(filteredTx) < TX_DEADBAND_DEG) return 0.0;

        return filteredTx;
    }

    /**
     * Returns true if a goal tag detection has been received within STALE_THRESHOLD_MS.
     * Use this to decide whether to trust getFilteredGoalBearing().
     */
    public boolean isGoalFresh() {
        return (System.currentTimeMillis() - lastGoalSeenMs) < STALE_THRESHOLD_MS;
    }

    /**
     * Deprecated alias kept for any callers that still use the old name.
     * Prefer getFilteredGoalBearing() for turret control.
     *
     * @deprecated
     */
    @Deprecated
    public double getGoalBearing() {
        return getRawGoalBearing();
    }

    public LLResultTypes.FiducialResult getGoalAT() {
        if (isGoalVisible()) {
            int targetID = RobotSubsystem.INSTANCE.getAllianceColor()
                    == RobotSubsystem.AllianceColor.BLUE ? 20 : 24;
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                if (fiducial.getFiducialId() == targetID) return fiducial;
            }
        }
        return null;
    }

    public double getDistanceToGoal() {
        // 10 deg camera pitch, 13.25" lens height, 29.5" goal height
        LLResultTypes.FiducialResult goalAT = getGoalAT();
        if (goalAT == null) return -99999;
        double angleToGoalDegrees = 10 + goalAT.getTargetYDegrees();
        double angleRad = angleToGoalDegrees * (Math.PI / 180.0);
        return (29.5 - 13.5) / Math.tan(angleRad);
    }

    /**
     * Returns true if the alliance-appropriate goal tag is currently visible.
     */
    public static boolean isGoalVisible() {
        boolean isBlue = RobotSubsystem.INSTANCE.getAllianceColor()
                == RobotSubsystem.AllianceColor.BLUE;
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
        if (goalID == 0) {
            goalID = RobotSubsystem.INSTANCE.getAllianceColor()
                    == RobotSubsystem.AllianceColor.RED ? 24 : 20;
        }
        getDetections();

        ActiveOpMode.telemetry().addData("Detection IDs: ", detectionIDs);
        ActiveOpMode.telemetry().addData("Goal Fresh: ", isGoalFresh());
        ActiveOpMode.telemetry().addData("Raw TX: ", getRawGoalBearing());
        ActiveOpMode.telemetry().addData("Filtered TX: ", filteredTx);
        ActiveOpMode.telemetry().addData("Output TX (deadbanded): ", getFilteredGoalBearing());
    }
}