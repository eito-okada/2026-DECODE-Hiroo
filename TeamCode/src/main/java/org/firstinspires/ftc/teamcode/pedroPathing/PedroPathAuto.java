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
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Same path as BlueGoalTracker (ApproachChain -> shot 1 @ (52,83) -> ShootChain -> ReturnChain ->
 * shot 2 @ (52,83)), but with the simplified motor controls from the current MecanumTeleOp:
 *   motor0 = intake      motor1/2/4/5 = LF/LB/RF/RB     motor6+7 = shooter      servo0 = gate
 *
 * Shooting has no RPM table or vision correction — the flywheel (motor6+7) and the intake
 * (motor0) both just run at a fixed power for the whole OpMode. To fire, the gate servo swings
 * from its 90 deg rest position to 0 deg (same rest/turned positions as MecanumTeleOp's transfer
 * gate) so the continuously-running intake pushes the staged balls through into the running
 * flywheel; after a 2 s dwell the gate returns to rest, closing the feed.
 *
 * Draws the robot on the Panels field and prints detailed telemetry.
 */
@Autonomous(name = "Pedro Path Auto (Fixed Shooter)", group = "Pedro Tests")
public class PedroPathAuto extends LinearOpMode {

    // Starting pose = start of MainChain, facing the blue goal (144 deg).
    private static final Pose START =
            new Pose(20.513313609467456, 121.19600591715977, Math.toRadians(144));

    // Intake motor (must match the Driver Station robot configuration).
    private static final String INTAKE_MOTOR_NAME = "motor0";
    private static final double INTAKE_POWER = 1.0;

    // Flywheel shooter: runs at a single fixed power for the entire OpMode.
    private static final String SHOOTER_A_NAME = "motor6";
    private static final String SHOOTER_B_NAME = "motor7";
    private static final double SHOOTER_POWER = 0.6;

    // Gate servo: 300 deg range, so position = angle / 300. Same rest/turned positions as
    // MecanumTeleOp's transfer gate (90 deg rest / 0 deg turned), no direction reversal.
    private static final String GATE_SERVO_NAME = "servo0";
    private static final double GATE_REST = 90.0 / 300.0;
    private static final double GATE_TURNED = 0.0 / 300.0;

    // How long to hold the gate open + intake spinning to let the launch happen.
    private static final long SHOT_DWELL_MS = 2000;

    private Follower follower;
    private TelemetryManager panels;
    private Paths paths;
    private DcMotor intake;
    private DcMotor shooterA, shooterB;
    private Servo gate;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START);

        intake = hardwareMap.get(DcMotor.class, INTAKE_MOTOR_NAME);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterA = hardwareMap.get(DcMotor.class, SHOOTER_A_NAME);
        shooterB = hardwareMap.get(DcMotor.class, SHOOTER_B_NAME);
        shooterA.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterB.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        gate = hardwareMap.get(Servo.class, GATE_SERVO_NAME);
        gate.setPosition(GATE_REST);

        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        Drawing.init();

        paths = new Paths(follower);

        // Shooter runs at all times, from init through the whole OpMode.
        shooterA.setPower(SHOOTER_POWER);
        shooterB.setPower(SHOOTER_POWER);

        while (opModeInInit()) {
            drawAndTelemetry("INIT — press START");
            sleep(20); // throttle the init loop so Panels/START stay responsive
        }
        if (isStopRequested()) {
            stopAll();
            return;
        }

        // Intake runs at all times for the rest of the OpMode, same as the shooter.
        intake.setPower(INTAKE_POWER);

        // Drive segs 1-3 up to the shooting spot (52,83).
        follower.followPath(paths.ApproachChain, true);
        while (opModeIsActive() && follower.isBusy()) {
            drawAndTelemetry("approach");
        }

        // Arrived at (52,83): fire, then drive seg 4. The flywheel keeps spinning throughout.
        fireShot("shot 1 @ (52,83)");

        follower.followPath(paths.ShootChain, true);
        while (opModeIsActive() && follower.isBusy()) {
            drawAndTelemetry("seg4");
        }

        // From (46,59): drive to (22,59) at 180 deg, then swing round to the shooting
        // spot (52,83) turning to 135 deg.
        follower.followPath(paths.ReturnChain, true);
        while (opModeIsActive() && follower.isBusy()) {
            drawAndTelemetry("return");
        }

        // Back at (52,83): fire the second shot the same way.
        fireShot("shot 2 @ (52,83)");

        // Done — hold the final pose.
        while (opModeIsActive()) {
            drawAndTelemetry("done — holding");
        }

        stopAll();
    }

    /**
     * Fires at the current (held) pose: swing the gate from 90 deg rest to 0 deg (turned) so the
     * staged balls (pushed by the continuously-running intake) launch into the running flywheel,
     * hold for SHOT_DWELL_MS, then swing the gate back to rest.
     */
    private void fireShot(String label) {
        gate.setPosition(GATE_TURNED);

        long dwellDeadline = System.currentTimeMillis() + SHOT_DWELL_MS;
        while (opModeIsActive() && System.currentTimeMillis() < dwellDeadline) {
            drawAndTelemetry(label + " — firing");
        }

        gate.setPosition(GATE_REST);
    }

    /** Stops the intake and shooter and returns the gate to rest. Call when the OpMode ends. */
    private void stopAll() {
        intake.setPower(0);
        shooterA.setPower(0);
        shooterB.setPower(0);
        gate.setPosition(GATE_REST);
    }

    /** One control iteration: update follower, print detailed telemetry, draw on field. */
    private void drawAndTelemetry(String phase) {
        follower.update();

        Pose p = follower.getPose();

        panels.debug("=== Pedro Path Auto (Fixed Shooter) ===");
        panels.debug("phase: " + phase);
        panels.debug("busy: " + follower.isBusy() + "   turning: " + follower.isTurning());
        panels.debug(String.format("shooter power: %.2f   gate: %.2f", SHOOTER_POWER, gate.getPosition()));

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

    /** The autonomous path, built once from the follower. Same geometry as BlueGoalTracker. */
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

            // Seg 4: after the shot at (52,83), drive to (46,59) while turning 135 -> 180 deg.
            // Separate chain so the robot can hold + fire first. Same early-turn treatment as
            // seg 1: (46,59) is where ReturnChain's intake leg begins, so finishing the rotation
            // at t=1 would start that leg mid-turn.
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
