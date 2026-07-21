package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Gamepad-tunable flywheel bench for finding the velocity PIDF and the distance->RPM table
 * values that Shooter uses. No odometry, no vision — you set a target RPM by hand, watch the
 * measured RPM recover as you feed balls, and read the final numbers off telemetry to paste
 * into Shooter's constants.
 *
 * Drives the same motors (motor6 + motor7) with the same velocity setup as Shooter and reuses
 * Shooter's RPM<->ticks/sec conversion, so what you tune here is exactly what runs in a match.
 *
 * Controls (gamepad1):
 *   A / B / X ........... select which coefficient the D-pad edits:  A=F  B=P  X=I
 *   Y ................... toggle intake (motor0) on / off
 *   D-pad up / down ..... increase / decrease the selected coefficient by the current step
 *   D-pad right / left .. step size x10 / /10  (coarse vs. fine)
 *   Right / left bumper . target RPM +/- 100
 *   Right / left trigger  target RPM +/- 10   (fine)
 *   Back (share) ........ target RPM = 0 (let it coast)
 * Coefficient edits are pushed to the motor immediately. D is fixed at Shooter's current
 * constant and is not tunable here.
 *
 * Controls (gamepad2):
 *   Right trigger ....... intake (motor0), proportional — lets you feed balls into the
 *                          spinning flywheel while tuning without touching gamepad1 at all.
 *                          Overridden by the gamepad1 X/Y toggle while it's on.
 */
@TeleOp(name = "Shooter Tuner", group = "Tuning")
public class ShooterTuner extends OpMode {

    private static final String FLYWHEEL_A_NAME = "motor6";
    private static final String FLYWHEEL_B_NAME = "motor7";
    private static final String INTAKE_MOTOR_NAME = "motor0";

    private DcMotorEx flywheelA, flywheelB;
    private DcMotor intake;

    // Live-editable PIDF, seeded from Shooter's current constants. D is fixed (not tuned here).
    private final double d = Shooter.VEL_D;
    private double p = Shooter.VEL_P, i = Shooter.VEL_I, f = Shooter.VEL_F;
    private double step = 1.0;        // coefficient edit step
    private double targetRpm = 0.0;
    private boolean intakeOn = false;

    private enum Coeff { F, P, I }
    private Coeff selected = Coeff.F;

    // Rising-edge latches so a held button acts once per press.
    private boolean prevA, prevB, prevX, prevY;
    private boolean prevUp, prevDown, prevLeft, prevRight;
    private boolean prevRb, prevLb, prevRt, prevLt, prevBack;

    @Override
    public void init() {
        flywheelA = hardwareMap.get(DcMotorEx.class, FLYWHEEL_A_NAME);
        flywheelB = hardwareMap.get(DcMotorEx.class, FLYWHEEL_B_NAME);

        for (DcMotorEx m : new DcMotorEx[]{flywheelA, flywheelB}) {
            m.setDirection(DcMotor.Direction.FORWARD); // match Shooter; flip BOTH if backward
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        flywheelA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (Shooter.BOTH_ENCODERS) {
            flywheelB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            flywheelB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            flywheelB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        pushPidf();

        intake = hardwareMap.get(DcMotor.class, INTAKE_MOTOR_NAME);
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Shooter Tuner ready. A=F B=P, dpad edits, bumpers/triggers set RPM.");
        telemetry.addLine("gamepad1 X/Y = intake toggle. gamepad2 right trigger = intake (proportional).");
    }

    @Override
    public void loop() {
        handleSelection();
        handleIntakeToggle();
        handleEdits();
        handleTargetRpm();

        // Toggle (gamepad1 X/Y) overrides the proportional trigger (gamepad2) while it's on.
        intake.setPower(intakeOn ? 1.0 : gamepad2.right_trigger);

        // Command the flywheel every loop so PIDF edits and RPM changes take effect live.
        double ticksPerSec = Shooter.rpmToTicksPerSec(targetRpm);
        flywheelA.setVelocity(ticksPerSec);
        if (Shooter.BOTH_ENCODERS) {
            flywheelB.setVelocity(ticksPerSec);
        } else {
            flywheelB.setPower(flywheelA.getPower());
        }

        double measA = Shooter.ticksPerSecToRpm(flywheelA.getVelocity());
        telemetry.addData("selected", selected + "   step=" + trim(step));
        telemetry.addData("PIDF", "P=%s  I=%s  D=%s  F=%s", trim(p), trim(i), trim(d), trim(f));
        telemetry.addData("target RPM", "%.0f", targetRpm);
        telemetry.addData("intake", "toggle=%s  gp2 RT=%.2f", intakeOn, gamepad2.right_trigger);
        telemetry.addData("measured RPM (A)", "%.0f   err=%.0f", measA, targetRpm - measA);
        if (Shooter.BOTH_ENCODERS) {
            telemetry.addData("measured RPM (B)", "%.0f", Shooter.ticksPerSecToRpm(flywheelB.getVelocity()));
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        flywheelA.setPower(0);
        flywheelB.setPower(0);
        intake.setPower(0);
    }

    private void handleSelection() {
        if (gamepad1.a && !prevA) selected = Coeff.F;
        if (gamepad1.b && !prevB) selected = Coeff.P;
        if (gamepad1.x && !prevX) selected = Coeff.I;
        prevA = gamepad1.a; prevB = gamepad1.b; prevX = gamepad1.x;
    }

    private void handleIntakeToggle() {
        if (gamepad1.y && !prevY) intakeOn = !intakeOn;
        prevY = gamepad1.y;
    }

    private void handleEdits() {
        if (gamepad1.dpad_right && !prevRight) step *= 10.0;
        if (gamepad1.dpad_left && !prevLeft) step = Math.max(0.001, step / 10.0);

        double delta = 0;
        if (gamepad1.dpad_up && !prevUp) delta = step;
        if (gamepad1.dpad_down && !prevDown) delta = -step;
        if (delta != 0) {
            switch (selected) {
                case F: f = Math.max(0, f + delta); break;
                case P: p = Math.max(0, p + delta); break;
                case I: i = Math.max(0, i + delta); break;
            }
            pushPidf();
        }
        prevUp = gamepad1.dpad_up; prevDown = gamepad1.dpad_down;
        prevLeft = gamepad1.dpad_left; prevRight = gamepad1.dpad_right;
    }

    private void handleTargetRpm() {
        boolean rt = gamepad1.right_trigger > 0.5;
        boolean lt = gamepad1.left_trigger > 0.5;
        if (gamepad1.right_bumper && !prevRb) targetRpm += 100;
        if (gamepad1.left_bumper && !prevLb) targetRpm = Math.max(0, targetRpm - 100);
        if (rt && !prevRt) targetRpm += 10;
        if (lt && !prevLt) targetRpm = Math.max(0, targetRpm - 10);
        if (gamepad1.back && !prevBack) targetRpm = 0;
        prevRb = gamepad1.right_bumper; prevLb = gamepad1.left_bumper;
        prevRt = rt; prevLt = lt; prevBack = gamepad1.back;
    }

    private void pushPidf() {
        flywheelA.setVelocityPIDFCoefficients(p, i, d, f);
        if (Shooter.BOTH_ENCODERS) flywheelB.setVelocityPIDFCoefficients(p, i, d, f);
    }

    /** Compact number formatting for telemetry (drops trailing zeros). */
    private static String trim(double v) {
        if (v == Math.rint(v)) return String.valueOf((long) v);
        return String.format("%.3f", v);
    }
}
