package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Eito ver. Simple code", group = "Main")
public class jan25 extends OpMode {

    // Drive motors
    DcMotor fl, fr, bl, br;

    // Mechanisms
    DcMotor intake, arm, launcher1, launcher2;

    // Launcher toggle
    boolean launcherOn = false;
    boolean lastLauncherBtn = false;

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

        // Mechanisms
        intake = hardwareMap.get(DcMotor.class, "motor4");
        arm = hardwareMap.get(DcMotor.class, "motor5");
        launcher1 = hardwareMap.get(DcMotor.class, "motor6");
        launcher2 = hardwareMap.get(DcMotor.class, "motor7");

        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher2.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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
        double dir = gamepad1.left_bumper ? -1.0 : 1.0;

        // Intake
        if (gamepad1.left_trigger > 0.5)
            intake.setPower(1.0 * dir);
        else
            intake.setPower(0);

        // Arm
        if (gamepad1.right_trigger > 0.5)
            arm.setPower(1.0 * dir);
        else
            arm.setPower(0);

        // Launcher toggle
        boolean launcherBtn = gamepad1.right_bumper;
        if (launcherBtn && !lastLauncherBtn)
            launcherOn = !launcherOn;
        lastLauncherBtn = launcherBtn;

        double launchPower = launcherOn ? 1.0 : 0.0;
        launcher1.setPower(launchPower);
        launcher2.setPower(launchPower);
    }
}
