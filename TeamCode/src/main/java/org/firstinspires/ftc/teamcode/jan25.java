package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Simple Mecanum + Shooter", group = "Main")
public class jan25 extends OpMode {

    // Drive motors
    DcMotor fl, fr, bl, br;

    // Mechanisms
    DcMotor intakeLeft, intakeRight;
    DcMotor transfer;
    DcMotor shooter;

    // Shooter toggle
    boolean shooterOn = false;
    boolean lastShooterBtn = false;

    static final double DEADZONE = 0.05;

    @Override
    public void init() {

        // Drive motors
        fl = hardwareMap.get(DcMotor.class, "motor0");
        bl = hardwareMap.get(DcMotor.class, "motor1");
        fr = hardwareMap.get(DcMotor.class, "motor2");
        br = hardwareMap.get(DcMotor.class, "motor3");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Mechanism motors
        intakeLeft  = hardwareMap.get(DcMotor.class, "motor4");
        intakeRight = hardwareMap.get(DcMotor.class, "motor5");
        transfer    = hardwareMap.get(DcMotor.class, "motor6");
        shooter     = hardwareMap.get(DcMotor.class, "motor7");

        // Directions (CHANGE if your robot runs backwards)
        intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Shooter usually coasts
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void loop() {

        // =========================
        // DRIVE (MECANUM)
        // =========================
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double r = gamepad1.right_stick_x;

        if (Math.abs(y) < DEADZONE) y = 0;
        if (Math.abs(x) < DEADZONE) x = 0;
        if (Math.abs(r) < DEADZONE) r = 0;

        double flPow = y + x + r;
        double frPow = y - x - r;
        double blPow = y - x + r;
        double brPow = y + x - r;

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

        // Intake + Transfer (hold)
        if (gamepad1.a) {
            intakeLeft.setPower(1.0);
            intakeRight.setPower(1.0);
            transfer.setPower(1.0);
        }
        // Reverse Intake + Transfer (hold)
        else if (gamepad1.b) {
            intakeLeft.setPower(-1.0);
            intakeRight.setPower(-1.0);
            transfer.setPower(-1.0);
        }
        else {
            intakeLeft.setPower(0);
            intakeRight.setPower(0);
            transfer.setPower(0);
        }

        // Shooter toggle
        boolean shooterBtn = gamepad1.right_bumper;
        if (shooterBtn && !lastShooterBtn)
            shooterOn = !shooterOn;
        lastShooterBtn = shooterBtn;

        shooter.setPower(shooterOn ? 1.0 : 0.0);
    }
}
