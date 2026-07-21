package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;

import java.util.Arrays;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Same paths as PedroPathTest (ApproachChain, ShootChain, ReturnChain, intake motor4 on the
 * intake legs) — PLUS the always-running flywheel shooter kept at the right RPM for the range,
 * with a vision-corrected distance. This is the vision-assisted variant: it arm()s the Shooter's
 * AprilTag sampler so, whenever goal tag 20 is in frame, the working distance (and therefore the
 * flywheel RPM) is corrected toward what the camera measures. PedroPathTest is the odometry-only
 * twin.
 *
 * The flywheel, transfer gate, and distance correction all live in Shooter, which MecanumTeleOp
 * shares, so every constant that needs measuring is tuned in exactly one place. shooter.update()
 * runs every loop right after follower.update() (inside drawAndTelemetry), so the flywheel holds
 * the correct RPM through every segment; each shot waits for shooter.atSpeed() before the gate opens.
 */
@Autonomous(name = "Blue Goal Tracker (Vision)", group = "Pedro Tests")
public class BlueGoalTracker extends LinearOpMode {

    // Starting pose = start of the path, facing the blue goal (144 deg). Identical to
    // PedroPathTest's START so the two OpModes agree.
    private static final Pose START =
            new Pose(20.513313609467456, 121.19600591715977, Math.toRadians(144));

    // Intake motor (must match the Driver Station robot configuration).
    private static final String INTAKE_MOTOR_NAME = "motor4";
    private static final double INTAKE_POWER = 1.0;

    // The start pose here is surveyed, so odometry is trusted at first and the initial vision
    // sample can wait out a normal correction interval.
    private static final long FIRST_SAMPLE_DELAY_MS = 3000;

    // How long to hold the transfer gate open at each shooting spot — long enough for the
    // always-spinning transfer wheels to feed all staged balls into the flywheel. TUNE.
    private static final long FEED_WINDOW_MS = 1200;
    // Cap how long we wait for the flywheel to reach speed before firing anyway, so an untuned
    // RPM target can't stall the routine forever.
    private static final long SPINUP_TIMEOUT_MS = 2500;

    private Follower follower;
    private TelemetryManager panels;
    private Paths paths;
    private DcMotor intake;
    private Shooter shooter;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START);

        intake = hardwareMap.get(DcMotor.class, INTAKE_MOTOR_NAME);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter = new Shooter(hardwareMap, follower);

        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        Drawing.init();

        paths = new Paths(follower);

        while (opModeInInit()) {
            drawAndTelemetry("INIT — press START");
            sleep(20); // throttle the init loop so Panels/START stay responsive
        }
        if (isStopRequested()) return;

        // First vision sample allowed a few seconds into the match (keeps the detector idle
        // through the whole INIT wait so it costs nothing until we're actually driving). The
        // flywheel is already spinning up from here via shooter.update() inside drawAndTelemetry.
        shooter.arm(FIRST_SAMPLE_DELAY_MS);

        // Drive segs 1-3 up to the shooting spot (52,83), running the intake on seg 2.
        follower.followPath(paths.ApproachChain, true);
        while (opModeIsActive() && follower.isBusy()) {
            // Seg 2 ((46,83) -> (22,83)) is chain index 1; run the intake only there.
            boolean onSeg2 = follower.getChainIndex() == 1;
            intake.setPower(onSeg2 ? INTAKE_POWER : 0);
            drawAndTelemetry(onSeg2 ? "seg2 (intaking)" : "approach");
        }
        intake.setPower(0);

        // Arrived at (52,83): hold the pose, wait for the flywheel to reach speed, then open
        // the transfer gate so the staged balls feed into the flywheel. drawAndTelemetry() keeps
        // calling follower.update(), so the robot holds here (PID active) rather than drifting.
        fireBurst("shot 1 @ (52,83)");

        // Shot done — drive seg 4. The flywheel keeps spinning the whole time (shooter.update()).
        follower.followPath(paths.ShootChain, true);
        while (opModeIsActive() && follower.isBusy()) {
            drawAndTelemetry("seg4");
        }

        // From (46,59): run the intake again while driving to (22,59) at 180 deg (chain
        // index 0), then swing round to the shooting spot (52,83) turning to 135 deg
        // (chain index 1) with the intake off.
        follower.followPath(paths.ReturnChain, true);
        while (opModeIsActive() && follower.isBusy()) {
            boolean onIntakeLeg = follower.getChainIndex() == 0;
            intake.setPower(onIntakeLeg ? INTAKE_POWER : 0);
            drawAndTelemetry(onIntakeLeg ? "return leg (intaking)" : "swing to (52,83)");
        }
        intake.setPower(0);

        // Back at (52,83): fire the second burst the same way.
        fireBurst("shot 2 @ (52,83)");

        // Done — hold the final pose.
        while (opModeIsActive()) {
            drawAndTelemetry("done — holding");
        }

        shooter.stop();
    }

    /**
     * Fires a burst at the current (held) pose: spin the flywheel up to speed (bounded by
     * SPINUP_TIMEOUT_MS so an untuned RPM can't stall us), open the transfer gate for
     * FEED_WINDOW_MS while the always-spinning transfer wheels feed the balls, then close it.
     * drawAndTelemetry() runs follower.update() throughout, so the robot holds its pose.
     */
    private void fireBurst(String label) {
        long spinupDeadline = System.currentTimeMillis() + SPINUP_TIMEOUT_MS;
        while (opModeIsActive() && !shooter.atSpeed()
                && System.currentTimeMillis() < spinupDeadline) {
            drawAndTelemetry(label + " — spinning up");
        }

        shooter.openGate();
        long feedDeadline = System.currentTimeMillis() + FEED_WINDOW_MS;
        while (opModeIsActive() && System.currentTimeMillis() < feedDeadline) {
            drawAndTelemetry(label + " — feeding");
        }
        shooter.closeGate();
    }

    /**
     * One control iteration: update follower, run the shooter (flywheel RPM + vision distance
     * correction), print detailed telemetry, draw on field. shooter.update() runs right after
     * follower.update() so the flywheel holds the correct RPM on every segment and during holds.
     */
    private void drawAndTelemetry(String phase) {
        follower.update();
        shooter.update(panels::debug);

        Pose p = follower.getPose();

        panels.debug("=== Blue Goal Tracker ===");
        panels.debug("phase: " + phase);
        panels.debug("busy: " + follower.isBusy() + "   turning: " + follower.isTurning());

        if (p != null) {
            panels.debug(String.format("pose:    x=%.1f  y=%.1f  h=%.1f deg",
                    p.getX(), p.getY(), Math.toDegrees(p.getHeading())));
        } else {
            panels.debug("pose: (initializing...)");
        }

        // Path-error metrics only exist while actively following a path.
        if (follower.isBusy()) {
            panels.debug(String.format("progress (t): %.2f", follower.getCurrentTValue()));

            Vector tErr = follower.getTranslationalError();
            if (tErr != null) {
                panels.debug(String.format("error: trans=%.2f in   heading=%.2f deg   drive=%.2f",
                        tErr.getMagnitude(), Math.toDegrees(follower.getHeadingError()), follower.getDriveError()));
            }
            Vector vel = follower.getVelocity();
            if (vel != null) {
                panels.debug(String.format("velocity: %.1f in/s", vel.getMagnitude()));
            }
        }
        // --- Vision correction status ---
        shooter.addTelemetry(panels::debug);
        panels.update(telemetry);

        // Draw ONLY the robot (null-safe); avoids drawDebug's path-point NPE.
        try {
            Drawing.drawRobot(follower.getPose());
            Drawing.sendPacket();
        } catch (Exception e) {
            panels.debug("draw skipped: " + e.getMessage());
            panels.update(telemetry);
        }
    }

    /**
     * The autonomous path, built once from the follower. Identical geometry to PedroPathTest:
     * ApproachChain (segs 1-3) to the shooting spot (52,83), then ShootChain (seg 4) to (46,59).
     */
    public static class Paths {

        // Fraction of a segment by which its heading turn must be COMPLETE, for the two segments
        // that feed an intake leg (seg 1 -> seg 2, seg 4 -> seg 5). The remaining (1 - this) of
        // the segment is spare distance for the heading PID to settle, so the robot enters the
        // intake leg already square at 180 deg and drives a genuinely straight line with the
        // intake mouth leading. Lower = turn finishes earlier and has longer to settle, but the
        // turn itself is more aggressive. TUNE: watch heading error at the corner on Panels and
        // lower this until the balls feed reliably / raise it if the turn starts hurting tracking.
        private static final double HEADING_SETTLE_T = 0.5;

        public PathChain ApproachChain;  // segs 1-3: drive to the shooting spot (52,83)
        public PathChain ShootChain;     // seg 4: leave the shooting spot for (46,59)
        public PathChain ReturnChain;    // segs 5-6: (46,59) -> (22,59) -> back to (52,83)

        public Paths(Follower follower) {
            ApproachChain = follower.pathBuilder()
                    // Seg 1: START -> corner (46,83), rounded so the robot ARRIVES
                    // heading west (180 deg) instead of hitting a sharp vertex.
                    //   C1 sits on the original START->corner line, so the robot
                    //     leaves START in the same direction as before.
                    //   C2 is placed east of the corner, so the curve's exit tangent
                    //     points west -> straight into seg 2 with no turn.
                    // Trade-off: the path bows slightly east of the corner (~x=50).
                    // Pull C2 toward (46,83) for a tighter corner, push it east for a
                    // wider/faster one.
                    .addPath(
                            new BezierCurve(Arrays.asList(
                                    new Pose(20.513, 121.196),  // start
                                    new Pose(33.260, 102.100),  // C1: midpoint of original seg 1
                                    new Pose(58.000, 83.000),   // C2: east of corner -> exits west
                                    new Pose(46.000, 83.000)    // corner
                            ))
                    )
                    // Reach 180 deg by t=HEADING_SETTLE_T rather than at the corner. The
                    // 2-arg overload defaults to finishing at t=1, which would leave the robot
                    // still rotating at the exact moment the chain hands off to the intake leg
                    // (interior chain segments advance purely on t-value — Pedro checks no
                    // heading/translational error there). Turning early leaves the rest of the
                    // curve for the heading PID to converge, so the robot enters seg 2 square
                    // at 180 deg and the intake leg is actually straight. Costs no time.
                    .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180),
                            HEADING_SETTLE_T)
                    // Seg 2: straight (46,83) -> (22,83), holding 180 deg so the robot
                    // drives forward with the intake leading. Chain index 1, so the
                    // intake gating in runOpMode still lines up with this leg.
                    .addPath(
                            new BezierLine(
                                    new Pose(46.000, 83.000),
                                    new Pose(22.000, 83.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    // Seg 3: straight (22,83) -> (52,83) while turning 180 -> 135 deg.
                    .addPath(
                            new BezierLine(
                                    new Pose(22.000, 83.000),
                                    new Pose(52.000, 83.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();

            // Seg 4: after the 1 s shot at (52,83), drive to (46,59) while turning
            // 135 -> 180 deg. Separate chain so the robot can hold + wait first.
            // Same early-turn treatment as seg 1: (46,59) is where ReturnChain's intake
            // leg begins, so finishing the rotation at t=1 would start that leg mid-turn.
            ShootChain = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(52.000, 83.000),
                                    new Pose(46.000, 59.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180),
                            HEADING_SETTLE_T)
                    .build();

            ReturnChain = follower.pathBuilder()
                    // Seg 5: straight (46,59) -> (22,59), holding 180 deg (intake leg).
                    .addPath(
                            new BezierLine(
                                    new Pose(46.000, 59.000),
                                    new Pose(22.000, 59.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    // Seg 6: (22,59) -> (52,83) while turning 180 -> 135 deg back to the
                    // shooting spot.
                    .addPath(
                            new BezierLine(
                                    new Pose(22.000, 59.000),
                                    new Pose(52.000, 83.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();
        }
    }
}
