package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "long press shooter (Fixed)", group = "Main")
public class jan25 extends OpMode {

    // Drive motors
    DcMotor fl, fr, bl, br;

    // Mechanisms
    DcMotor intakeLeft, intakeRight;
    DcMotor transfer;
    DcMotor shooter;

    // Toggles & Trackers
    boolean shooterOn = false;
    boolean lastShooterBtn = false;

    boolean flMotorOk = true;
    boolean lastDpadUp = false; // Added to fix dpad logic

    static final double DEADZONE = 0.05;

    @Override
    public void init() {

        // 1. HARDWARE MAPPING
        fl = hardwareMap.get(DcMotor.class, "motor0");
        bl = hardwareMap.get(DcMotor.class, "motor1");
        fr = hardwareMap.get(DcMotor.class, "motor2");
        br = hardwareMap.get(DcMotor.class, "motor3");

        // 2. DIRECTION FIX (Mecanum)
        // Left side = REVERSE, Right side = FORWARD
        fl.setDirection(DcMotorSimple.Direction.REVERSE); // UNCOMMENTED THIS FOR YOU
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        // 3. BRAKE BEHAVIOR
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 4. MECHANISMS
        intakeLeft = hardwareMap.get(DcMotor.class, "motor4");
        intakeRight = hardwareMap.get(DcMotor.class, "motor5");
        transfer = hardwareMap.get(DcMotor.class, "motor6");
        shooter = hardwareMap.get(DcMotor.class, "motor7");

        // Mechanism Directions
        intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);

        // Shooter usually coasts
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void loop() {

        // =========================
        // DRIVE (MECANUM)
        // =========================
        double y = -gamepad1.left_stick_y; // Remember, Y is reversed on gamepads
        double x = gamepad1.left_stick_x;
        double r = gamepad1.right_stick_x;

        if (Math.abs(y) < DEADZONE) y = 0;
        if (Math.abs(x) < DEADZONE) x = 0;
        if (Math.abs(r) < DEADZONE) r = 0;

        // Standard Mecanum Formulas
        double flPow = y + x + r;
        double frPow = y - x - r;
        double blPow = y - x + r;
        double brPow = y + x - r;

        // Normalize speeds (keep them under 1.0)
        double max = Math.max(1.0,
                Math.max(Math.abs(flPow),
                        Math.max(Math.abs(frPow),
                                Math.max(Math.abs(blPow), Math.abs(brPow)))));

        fl.setPower(flPow / max);
        fr.setPower(frPow / max);
        bl.setPower(blPow / max);
        br.setPower(brPow / max);

        // =========================
        // MECHANISMS
        // =========================

        boolean reverse = gamepad1.left_bumper; // L1
        double dir = reverse ? -1.0 : 1.0;

        // ---- Intake (Hold A) ----
        if (gamepad1.a) {
            intakeLeft.setPower(1.0 * dir);
            intakeRight.setPower(1.0 * dir);
        } else {
            intakeLeft.setPower(0);
            intakeRight.setPower(0);
        }

        // ---- Transfer (Hold X) ----
        // Safety: If shooter is OFF and we aren't reversing, don't run transfer?
        // (Removing the safety check for now so you can test freely, or uncomment below)
        // if (!shooterOn && !reverse) transfer.setPower(0);

        if (gamepad1.x) {
            transfer.setPower(1.0 * dir);
        } else {
            transfer.setPower(0);
        }

        // ---- Debug: FL Direction Toggle (DPAD UP) ----
        // Fixed: dpadUpWasPressed() does not exist in standard OpMode.
        // Replaced with standard "Just Pressed" logic.
        boolean currentDpadUp = gamepad1.dpad_up;
        if (currentDpadUp && !lastDpadUp) {
            if (flMotorOk) {
                fl.setDirection(DcMotorSimple.Direction.REVERSE);
                flMotorOk = false;
            } else {
                fl.setDirection(DcMotorSimple.Direction.FORWARD);
                flMotorOk = true;
            }
        }
        lastDpadUp = currentDpadUp;

        // ---- Shooter Toggle (R1) ----
        // Fixed: Completed the cut-off logic here
        boolean shooterBtn = gamepad1.right_bumper;

        if (shooterBtn && !lastShooterBtn) {
            shooterOn = !shooterOn; // Toggle ON/OFF
        }
        lastShooterBtn = shooterBtn;

        shooter.setPower(shooterOn ? 1.0 : 0.0);
    }
}