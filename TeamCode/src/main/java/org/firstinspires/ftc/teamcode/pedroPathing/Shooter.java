package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * The flywheel shooter (motor6 + motor7) plus the transfer-stopper gate (servo0),
 * driven from the robot's distance to the blue goal. Shared by the teleop and the two
 * autonomous OpModes so every constant that needs measuring lives in exactly one place.
 *
 * How it works each loop:
 *   1. Occasionally (when the goal AprilTag is in frame) correct the odometry-derived
 *      distance toward the tag's measured range. Vision is OFF the rest of the time, so
 *      it costs almost nothing — same non-blocking sampler the old turret code used.
 *   2. Look up a target flywheel RPM for that distance from DIST_RPM_TABLE.
 *   3. Command the flywheel with closed-loop VELOCITY control (RUN_USING_ENCODER +
 *      setVelocity). The built-in velocity PIDF is what makes the wheel recover RPM
 *      quickly after each ball robs it of speed — the whole reason we shoot 3 in a row.
 *
 * The flywheel is meant to spin CONTINUOUSLY. The transfer wheels share a chain with the
 * intake and always turn, so a ball would feed into the running flywheel on its own; the
 * gate servo (servo0) holds a cover over the transfer to block that until the driver
 * opens it to shoot.
 *
 * The caller owns the Follower and must keep its pose fresh — call follower.update()
 * (auto) or follower.updatePose() (teleop) BEFORE calling update() here.
 */
public class Shooter {

    /** Sink for status lines, so each OpMode can route them to its own telemetry. */
    public interface Logger {
        void log(String line);
    }

    // --- Field geometry -----------------------------------------------------------------
    // Blue goal position in Pedro field coordinates (inches). Carried over from the old
    // GoalTurret so distances stay consistent with the autonomous paths. SURVEY on the real
    // field and replace if needed.
    public static final double GOAL_X = 0.0;
    public static final double GOAL_Y = 136.0;

    // --- Flywheel motors ----------------------------------------------------------------
    // motor6 + motor7 share one chain with the flywheel and turn the SAME direction. We
    // close the velocity loop on motor6's encoder; motor7 mirrors motor6's applied power
    // (see BOTH_ENCODERS). Names must match the Driver Station robot configuration.
    private static final String FLYWHEEL_A_NAME = "motor6"; // primary — carries the encoder
    private static final String FLYWHEEL_B_NAME = "motor7"; // secondary — mirrors motor6

    // Set true ONLY if motor7 also has an encoder cable plugged in; then it runs its own
    // setVelocity loop instead of mirroring motor6's power. Default false: one encoder.
    public static final boolean BOTH_ENCODERS = false;

    // Encoder ticks per flywheel-motor revolution. MEASURE THIS for your motor (e.g. a bare
    // GoBILDA 5203 yellow-jacket has 28 ticks/rev at the motor, a 6000rpm bare motor 28,
    // etc. — read your motor's spec or spin it a known number of turns and read the delta).
    // Used to convert the RPM setpoints below into the ticks/sec that setVelocity wants.
    public static final double TICKS_PER_REV = 28.0;

    // --- Distance -> target RPM lookup --------------------------------------------------
    // Each row is {distanceToGoal (inches), flywheel RPM}. update() linearly interpolates
    // between rows and clamps outside the ends. MUST be sorted by distance ascending.
    // TUNE these on the real robot: park at a known distance, use ShooterTuner to find the
    // RPM that scores, and record the pair here. Add as many rows as you like.
    public static final double[][] DIST_RPM_TABLE = {
            {40.0, 2600.0},
            {80.0, 3200.0},
            {120.0, 3800.0},
            {160.0, 4400.0},
    };

    // --- Velocity PIDF ------------------------------------------------------------------
    // Applied to motor6 (and motor7 if BOTH_ENCODERS). Verified with ShooterTuner: these
    // defaults held 3000 RPM cleanly, no edits needed. F dominates for a flywheel; P/D
    // help recovery after a ball. Re-tune if the flywheel, gearing, or motor changes.
    public static final double VEL_P = 10.0;
    public static final double VEL_I = 0.0;
    public static final double VEL_D = 0.0;
    public static final double VEL_F = 14.0;

    // The wheel counts as "up to speed" (atSpeed()) when measured RPM is within this many
    // RPM of the target. Gates auto shots so we don't fire during spin-up or recovery.
    public static final double RPM_TOLERANCE = 120.0;

    // --- Transfer-stopper gate (servo0) -------------------------------------------------
    // The cover is on a 1:1 gear; 180 deg of travel swings it clear. On a 300 deg servo,
    // 180 deg = 0.6 of full travel. CLOSED blocks the transfer; OPEN lets balls feed.
    // Carried over from the old shooter-servo constants. If it moves the wrong way, swap
    // these two values or set the servo direction.
    private static final String GATE_SERVO_NAME = "servo0";
    private static final double GATE_CLOSED = 0.0;
    private static final double GATE_OPEN = 0.6; // 180 deg on a 300 deg servo

    // --- AprilTag distance correction ---------------------------------------------------
    private static final String WEBCAM_NAME = "Webcam 1";       // match DS config
    private static final int GOAL_TAG_ID = 20;                  // blue goal DECODE tag
    private static final long CORRECTION_INTERVAL_MS = 3000;    // "every few seconds"
    private static final long VISION_WARMUP_MS = 400;           // let a fresh frame land after enabling
    private static final double CORRECTION_GAIN = 0.3;          // fraction of the range error we absorb per sample
    private static final double MAX_CORRECTION_IN = 24.0;       // clamp: ignore implausibly large single jumps

    private final Follower follower;
    private final DcMotorEx flywheelA;
    private final DcMotorEx flywheelB;
    private final Servo gate;

    // Vision: detector + portal, and a tiny non-blocking sampler state machine (identical
    // shape to the old GoalTurret sampler).
    private final AprilTagProcessor aprilTag;
    private final VisionPortal visionPortal;
    private boolean sampling;                     // true while the detector is on for a sample window
    private long nextSampleTime = Long.MAX_VALUE; // wall-clock (ms) when the next sample may start
    private long sampleReadyTime;                 // wall-clock (ms) when the enabled detector should have a frame
    private boolean lastTagVisible;               // whether the most recent sample saw the goal tag

    // Correction added to the odometry distance, nudged toward the tag's measured range.
    private double distanceOffsetIn;
    private double lastCorrectionIn;
    private long lastCorrectionTime;

    // Cached each update() for telemetry / atSpeed().
    private double lastWorkingDistanceIn = -1;
    private double lastTargetRpm;

    public Shooter(HardwareMap hardwareMap, Follower follower) {
        this.follower = follower;

        flywheelA = hardwareMap.get(DcMotorEx.class, FLYWHEEL_A_NAME);
        flywheelB = hardwareMap.get(DcMotorEx.class, FLYWHEEL_B_NAME);
        configureFlywheel(flywheelA, true);
        configureFlywheel(flywheelB, BOTH_ENCODERS);

        gate = hardwareMap.get(Servo.class, GATE_SERVO_NAME);
        closeGate();

        // Same processor setup as the old vision code (cm / degrees).
        aprilTag = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                .addProcessor(aprilTag)
                .build();
        visionPortal.setProcessorEnabled(aprilTag, false); // idle until arm()
    }

    /** Both motors: same direction (chained), FLOAT so the flywheel coasts, and either a
     *  closed velocity loop (encoder present) or plain open-loop power (mirrored later). */
    private void configureFlywheel(DcMotorEx motor, boolean hasEncoder) {
        motor.setDirection(DcMotor.Direction.FORWARD); // flip BOTH together if the wheel runs backward
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        if (hasEncoder) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            applyPidf(motor);
        } else {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    /** Applies the velocity PIDF constants to a motor. Exposed via reconfigurePidf() so the
     *  tuner can push live edits, but production code just uses the constants above. */
    private static void applyPidf(DcMotorEx motor) {
        motor.setVelocityPIDFCoefficients(VEL_P, VEL_I, VEL_D, VEL_F);
    }

    // --- RPM <-> encoder velocity helpers (shared with ShooterTuner) --------------------

    /** Converts flywheel RPM to the ticks/sec that DcMotorEx.setVelocity expects. */
    public static double rpmToTicksPerSec(double rpm) {
        return rpm / 60.0 * TICKS_PER_REV;
    }

    /** Converts a measured ticks/sec (getVelocity) back to flywheel RPM. */
    public static double ticksPerSecToRpm(double ticksPerSec) {
        return ticksPerSec / TICKS_PER_REV * 60.0;
    }

    /**
     * Allows the first vision sample delayMs from now. Call once at START. Leaving the
     * sampler parked through INIT keeps the detector idle (and free) until we're playing.
     * OpModes that don't want vision correction (odometry-only) simply never call this.
     */
    public void arm(long delayMs) {
        nextSampleTime = System.currentTimeMillis() + delayMs;
    }

    /**
     * One control iteration: take a vision distance sample if due, then command the
     * flywheel to the distance-derived target RPM. Call once per loop AFTER the caller has
     * refreshed the follower's pose. Nothing here blocks.
     */
    public void update(Logger log) {
        updateVisionCorrection(log);

        double distance = workingDistance();
        lastWorkingDistanceIn = distance;
        double targetRpm = (distance < 0) ? 0 : rpmForDistance(distance);
        lastTargetRpm = targetRpm;

        commandRpm(targetRpm);

        if (distance >= 0) {
            log.log(String.format("shooter: dist=%.1f in  targetRPM=%.0f  measRPM=%.0f  %s",
                    distance, targetRpm, measuredRpm(), atSpeed() ? "AT SPEED" : "spinning up"));
        } else {
            log.log("shooter: waiting for pose");
        }
    }

    /** Commands both flywheel motors toward a target RPM (0 = let it coast). */
    public void commandRpm(double targetRpm) {
        double ticksPerSec = rpmToTicksPerSec(targetRpm);
        flywheelA.setVelocity(ticksPerSec);
        if (BOTH_ENCODERS) {
            flywheelB.setVelocity(ticksPerSec);
        } else {
            // No encoder on B: mirror the power the velocity controller settled on for A so
            // both motors share the load on the common chain.
            flywheelB.setPower(flywheelA.getPower());
        }
    }

    /** Opens the transfer cover so balls feed into the running flywheel. */
    public void openGate() {
        gate.setPosition(GATE_OPEN);
    }

    /** Closes the transfer cover so no ball reaches the flywheel. */
    public void closeGate() {
        gate.setPosition(GATE_CLOSED);
    }

    /** True once the flywheel is within RPM_TOLERANCE of its target — safe to feed a ball. */
    public boolean atSpeed() {
        if (lastTargetRpm <= 0) return false;
        return Math.abs(measuredRpm() - lastTargetRpm) <= RPM_TOLERANCE;
    }

    /** Measured flywheel RPM from motor6's encoder. */
    public double measuredRpm() {
        return ticksPerSecToRpm(flywheelA.getVelocity());
    }

    /** Straight-line distance (inches) from the current pose to the goal, corrected by the
     *  latest vision nudge, or -1 if there's no pose yet. */
    public double workingDistance() {
        double odo = odometryDistance();
        if (odo < 0) return -1;
        return Math.max(0, odo + distanceOffsetIn);
    }

    /** Raw odometry distance to the goal (no vision correction), or -1 if no pose yet. */
    public double odometryDistance() {
        Pose pose = follower.getPose();
        if (pose == null) return -1;
        return Math.hypot(GOAL_X - pose.getX(), GOAL_Y - pose.getY());
    }

    /** Linearly interpolates DIST_RPM_TABLE, clamped to its endpoints. */
    public static double rpmForDistance(double distanceIn) {
        double[][] t = DIST_RPM_TABLE;
        if (distanceIn <= t[0][0]) return t[0][1];
        if (distanceIn >= t[t.length - 1][0]) return t[t.length - 1][1];
        for (int i = 1; i < t.length; i++) {
            if (distanceIn <= t[i][0]) {
                double d0 = t[i - 1][0], r0 = t[i - 1][1];
                double d1 = t[i][0], r1 = t[i][1];
                double frac = (distanceIn - d0) / (d1 - d0);
                return r0 + frac * (r1 - r0);
            }
        }
        return t[t.length - 1][1]; // unreachable, satisfies the compiler
    }

    /** Prints the vision sampler's status. Separate from update() so callers order telemetry. */
    public void addTelemetry(Logger log) {
        if (sampling) {
            log.log("vision: SAMPLING (detector on)");
        } else {
            long secs = Math.max(0, nextSampleTime - System.currentTimeMillis()) / 1000;
            log.log(nextSampleTime == Long.MAX_VALUE
                    ? "vision: idle (not armed — odometry only)"
                    : String.format("vision: idle — next sample in %ds", secs));
        }
        log.log("vision: last sample " + (lastTagVisible ? "saw goal tag" : "no tag"));
        log.log(String.format("vision: distance offset %.1f in", distanceOffsetIn));
        if (lastCorrectionTime > 0) {
            long ago = (System.currentTimeMillis() - lastCorrectionTime) / 1000;
            log.log(String.format("vision: last dist nudge %.1f in (%ds ago)", lastCorrectionIn, ago));
        }
    }

    /** Stops the flywheel, closes the gate, and releases the camera. Call when the OpMode ends. */
    public void stop() {
        flywheelA.setPower(0);
        flywheelB.setPower(0);
        closeGate();
        if (visionPortal != null) visionPortal.close();
    }

    // --- internals ----------------------------------------------------------------------

    /**
     * Non-blocking sampler run once per loop. Most loops it does nothing. Every
     * CORRECTION_INTERVAL_MS it turns the detector on briefly, and once a fresh frame has
     * landed it reads the goal tag's range (if visible) and nudges the distance offset so
     * the working distance drifts toward what vision measures.
     */
    private void updateVisionCorrection(Logger log) {
        long now = System.currentTimeMillis();

        if (!sampling) {
            if (now >= nextSampleTime) {
                visionPortal.setProcessorEnabled(aprilTag, true);
                sampling = true;
                sampleReadyTime = now + VISION_WARMUP_MS;
            }
            return;
        }

        if (now < sampleReadyTime) return; // let the detector produce a frame

        AprilTagDetection tag = findTargetTag();
        lastTagVisible = tag != null && tag.ftcPose != null;
        if (lastTagVisible) {
            applyDistanceCorrection(tag, log);
        }

        visionPortal.setProcessorEnabled(aprilTag, false);
        sampling = false;
        nextSampleTime = now + CORRECTION_INTERVAL_MS;
    }

    private AprilTagDetection findTargetTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.id == GOAL_TAG_ID) {
                return detection;
            }
        }
        return null;
    }

    /**
     * Nudges distanceOffsetIn so the working distance moves a fraction of the way toward the
     * tag's measured range. Same spirit as the old heading correction: a small gain plus a
     * clamp, so one bad read can't yank the setpoint. ftcPose.range is in cm (we set the
     * processor to CM), so convert to inches first.
     */
    private void applyDistanceCorrection(AprilTagDetection tag, Logger log) {
        double odo = odometryDistance();
        if (odo < 0) return;

        double rangeIn = tag.ftcPose.range / 2.54; // cm -> in
        double target = rangeIn - odo;              // offset that would make working distance == vision range
        double delta = (target - distanceOffsetIn) * CORRECTION_GAIN;
        delta = Math.max(-MAX_CORRECTION_IN, Math.min(MAX_CORRECTION_IN, delta));

        distanceOffsetIn += delta;
        lastCorrectionIn = delta;
        lastCorrectionTime = System.currentTimeMillis();
    }
}
