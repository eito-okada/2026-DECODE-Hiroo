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
 * DECODE blue-goal simulation, built as a single Pedro PathChain (MainChain):
 *   Seg 1: (20.513, 121.196) -> (46, 83), heading interpolates 144 -> 180 deg
 *   Seg 2: (46, 83) -> (22, 83),          heading constant 0 deg
 *
 * Draws the robot on the Panels field and prints detailed telemetry.
 */
@Autonomous(name = "Pedro Path Test", group = "Pedro Tests")
public class PedroPathTest extends LinearOpMode {

    // Starting pose = start of MainChain, facing the blue goal (144 deg).
    private static final Pose START =
            new Pose(20.513313609467456, 121.19600591715977, Math.toRadians(144));

    // Intake motor (must match the Driver Station robot configuration).
    private static final String INTAKE_MOTOR_NAME = "motor4";
    private static final double INTAKE_POWER = 1.0;

    // Runs alongside the intake: starts the moment the intake first spins (seg 2)
    // and keeps running until the OpMode is stopped (the SDK zeroes it on stop).
    private static final String MOTOR6_NAME = "motor6";
    private static final double MOTOR6_POWER = 1.0;

    // Shooter servo (must match the Driver Station robot configuration).
    // 300 deg range servo, so 180 deg = 180/300 = 0.6 of full travel:
    //   0.0 = original/rest position, 0.6 = swept 180 deg (the shot).
    // If it turns the wrong way, swap these two values or set servo direction.
    private static final String SHOOTER_SERVO_NAME = "servo0";
    private static final double SHOOTER_REST = 0.0;
    private static final double SHOOTER_SHOT = 0.6; // 180 deg on a 300 deg servo

    private Follower follower;
    private TelemetryManager panels;
    private Paths paths;
    private DcMotor intake;
    private DcMotor motor6;
    private Servo shooter;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START);

        intake = hardwareMap.get(DcMotor.class, INTAKE_MOTOR_NAME);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor6 = hardwareMap.get(DcMotor.class, MOTOR6_NAME);
        motor6.setDirection(DcMotorSimple.Direction.FORWARD);
        motor6.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter = hardwareMap.get(Servo.class, SHOOTER_SERVO_NAME);
        shooter.setPosition(SHOOTER_REST); // start at the original position

        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        Drawing.init();

        paths = new Paths(follower);

        while (opModeInInit()) {
            drawAndTelemetry("INIT — press START");
            sleep(20); // throttle the init loop so Panels/START stay responsive
        }
        if (isStopRequested()) return;

        // Drive segs 1-3 up to the shooting spot (52,83), running the intake on seg 2.
        follower.followPath(paths.ApproachChain, true);
        while (opModeIsActive() && follower.isBusy()) {
            // Seg 2 ((46,83) -> (22,83)) is chain index 1; run the intake only there.
            boolean onSeg2 = follower.getChainIndex() == 1;
            intake.setPower(onSeg2 ? INTAKE_POWER : 0);
            // motor6 starts with the intake and is never zeroed after — it keeps
            // running through shooting, seg 4, and the final hold until stop.
            if (onSeg2) motor6.setPower(MOTOR6_POWER);
            drawAndTelemetry(onSeg2 ? "seg2 (intake + motor6 running)" : "approach");
        }
        intake.setPower(0);

        // Arrived at (52,83): sweep the shooter 180 deg to fire, then hold the pose
        // and wait 1 s while parked. drawAndTelemetry() keeps calling
        // follower.update(), so holdEnd keeps the robot here (PID active) rather
        // than drifting during the wait.
        shooter.setPosition(SHOOTER_SHOT);
        long shootDeadline = System.currentTimeMillis() + 1000;
        while (opModeIsActive() && System.currentTimeMillis() < shootDeadline) {
            drawAndTelemetry("shooting — hold 1s @ (52,83)");
        }

        // Shot done — start driving seg 4 AND return the servo at the same time.
        // setPosition() is non-blocking, so the robot drives away immediately while
        // the servo sweeps back on its own; nothing waits for it to finish.
        follower.followPath(paths.ShootChain, true);
        shooter.setPosition(SHOOTER_REST);
        while (opModeIsActive() && follower.isBusy()) {
            drawAndTelemetry("seg4 (servo returning)");
        }

        // Done — hold the final pose.
        while (opModeIsActive()) {
            drawAndTelemetry("done — holding");
        }
    }

    /** One control iteration: update follower, print detailed telemetry, draw on field. */
    private void drawAndTelemetry(String phase) {
        follower.update();

        Pose p = follower.getPose();

        panels.debug("=== Pedro Path Test ===");
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

    /** The autonomous path, built once from the follower. */
    public static class Paths {
        public PathChain ApproachChain;  // segs 1-3: drive to the shooting spot (52,83)
        public PathChain ShootChain;     // seg 4: leave the shooting spot for (46,59)

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
                    .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
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
            ShootChain = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(52.000, 83.000),
                                    new Pose(46.000, 59.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();
        }
    }
}
