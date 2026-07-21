package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Bench-test TeleOp for the shooter assembly, wired for the TEST rig (not the match robot):
 *   motor0 = intake
 *   motor6 = shooter 1   (both shooters run the SAME direction — shared chain)
 *   motor7 = shooter 2
 *   servo0 = stopper — swings from 90 to 180 degrees each time A is pressed (toggles between the two ends)
 *
 * Controls (gamepad1):
 *   right trigger ....... intake (analog — trigger amount = power)
 *   right bumper ........ hold to run both shooters
 *   dpad up / down ...... shooter power +/- 0.05 (so you can find a workable open-loop power)
 *   A ................... toggle servo0 between 90 deg (rest) and 180 deg (turned)
 */
@TeleOp(name = "Hardware Test", group = "Test")
public class HardwareTestTeleOp extends OpMode {

    // 300-degree servo: position = angle / 300. Rest is 90 deg; A swings the OTHER
    // way from before — UP to 180 deg instead of down toward 0 — so both ends sit
    // comfortably inside the servo's mechanical range with no clipping.
    private static final double SERVO_REST = 90.0 / 300.0;
    private static final double SERVO_TURNED = 180.0 / 300.0;

    private DcMotor intake;
    private DcMotor shooter1, shooter2;
    private Servo stopper;

    private double shooterPower = 1.0;
    private boolean servoTurned = false;

    // Rising-edge latches so a held button acts once per press.
    private boolean prevA, prevUp, prevDown;

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotor.class, "motor0");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter1 = hardwareMap.get(DcMotor.class, "motor6");
        shooter2 = hardwareMap.get(DcMotor.class, "motor7");
        // Same direction on both — they share one chain. Flip BOTH if the wheel runs backward.
        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);
        // FLOAT so the flywheel coasts down instead of braking hard.
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        stopper = hardwareMap.get(Servo.class, "servo0");
        stopper.setPosition(SERVO_REST);

        telemetry.addLine("Hardware test ready.");
        telemetry.addLine("RT = intake, RB = shooters, dpad = shooter power, A = servo 90/180 deg.");
    }

    @Override
    public void loop() {
        // Intake: analog on the right trigger.
        intake.setPower(gamepad1.right_trigger);

        // Shooter power tweak on the dpad (once per press).
        if (gamepad1.dpad_up && !prevUp) shooterPower = Math.min(1.0, shooterPower + 0.05);
        if (gamepad1.dpad_down && !prevDown) shooterPower = Math.max(0.0, shooterPower - 0.05);
        prevUp = gamepad1.dpad_up;
        prevDown = gamepad1.dpad_down;

        // Shooters: hold right bumper to spin both, same direction.
        double applied = gamepad1.right_bumper ? shooterPower : 0;
        shooter1.setPower(applied);
        shooter2.setPower(applied);

        // Servo: each A press toggles between 90 deg (rest) and 180 deg (turned).
        if (gamepad1.a && !prevA) {
            servoTurned = !servoTurned;
            stopper.setPosition(servoTurned ? SERVO_TURNED : SERVO_REST);
        }
        prevA = gamepad1.a;

        telemetry.addData("intake (RT)", "%.2f", gamepad1.right_trigger);
        telemetry.addData("shooters", "%s @ %.2f", gamepad1.right_bumper ? "ON" : "off", shooterPower);
        telemetry.addData("servo0", servoTurned ? "TURNED (180 deg)" : "rest (90 deg)");
        telemetry.update();
    }

    @Override
    public void stop() {
        intake.setPower(0);
        shooter1.setPower(0);
        shooter2.setPower(0);
    }
}
