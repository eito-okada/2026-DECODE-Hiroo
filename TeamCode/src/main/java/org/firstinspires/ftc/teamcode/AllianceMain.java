package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name = "Alliance Main + Dual Servos", group = "Alliance")
public class AllianceMain extends OpMode {

    // ==========================================
    // ポート設定メモ (Config Names)
    // ==========================================
    // Main Hub Motor 0: "motor0" (右後ろ br)
    // Main Hub Motor 1: "motor1" (右前 fr)
    // Main Hub Motor 2: "motor2" (左後ろ bl)
    // Ext  Hub Motor 0: "motor3" (射出 launcher)
    // Ext  Hub Motor 1: "motor4" (左前 fl)
    //
    // Ext  Hub Servo 0: "servo0" (Extension用)
    // Main Hub Servo 0: "servo1" (Main用)
    // ==========================================

    // --- DRIVE MOTORS ---
    private DcMotor fl, fr, bl, br;
    private boolean flOk, frOk, blOk, brOk;

    // --- MECHANISM MOTORS ---
    private DcMotor launcher;
    private boolean launcherOk;

    // --- SERVOS ---
    private Servo extServo;  // 反時計回り (CCW)
    private Servo mainServo; // 時計回り (CW)
    private boolean extServoOk, mainServoOk;

    // --- TOGGLE VARIABLES ---
    private boolean launcherActive = false;
    private boolean lastLauncherBtn = false;

    // サーボ用トグル変数（1つで両方を管理）
    private boolean servosActive = false;
    private boolean lastServoBtn = false;

    // --- SETTINGS ---
    private static final double DEADZONE = 0.05;
    private static final double POWER_LAUNCH = 1.0;

    // サーボの位置設定 (0.0 ～ 1.0)
    private static final double SERVO_POS_REST = 0.0;
    private static final double SERVO_POS_ACTIVE = 1.0;

    // スローモード設定
    private static final double SPEED_NORMAL = 1.0;
    private static final double SPEED_SLOW = 0.4;

    private IMU imu;

    @Override
    public void init() {
        // 1. IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        imu.resetYaw();

        // 2. Drive Motors
        try { br = hardwareMap.get(DcMotor.class, "motor0"); br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); br.setDirection(DcMotorSimple.Direction.FORWARD); br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); brOk = true; } catch (Exception e) {}
        try { fr = hardwareMap.get(DcMotor.class, "motor1"); fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); fr.setDirection(DcMotorSimple.Direction.FORWARD); fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); frOk = true; } catch (Exception e) {}
        try { bl = hardwareMap.get(DcMotor.class, "motor2"); bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); bl.setDirection(DcMotorSimple.Direction.REVERSE); bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); blOk = true; } catch (Exception e) {}
        try { fl = hardwareMap.get(DcMotor.class, "motor4"); fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); fl.setDirection(DcMotorSimple.Direction.REVERSE); fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); flOk = true; } catch (Exception e) {}

        // 3. Mechanism Motors
        try {
            launcher = hardwareMap.get(DcMotor.class, "motor3");
            launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            launcher.setDirection(DcMotorSimple.Direction.FORWARD);
            launcherOk = true;
        } catch (Exception e) { telemetry.addLine("Error: Launcher (motor3) not found"); }

        // 4. Servos

        // --- Extension Hub Servo (Port 0) : 反時計回り (CCW) ---
        try {
            extServo = hardwareMap.get(Servo.class, "servo0");
            // Forwardで0→1に動かすと反時計回り(CCW)
            extServo.setDirection(Servo.Direction.FORWARD);
            extServo.setPosition(SERVO_POS_REST);
            extServoOk = true;
        } catch (Exception e) { telemetry.addLine("Error: Ext Servo (servo0) not found"); }

        // --- Main Hub Servo (Port 0) : 時計回り (CW) ---
        try {
            mainServo = hardwareMap.get(Servo.class, "servo1");
            // Reverseで0→1に動かすと時計回り(CW)
            mainServo.setDirection(Servo.Direction.REVERSE);
            mainServo.setPosition(SERVO_POS_REST);
            mainServoOk = true;
        } catch (Exception e) { telemetry.addLine("Error: Main Servo (servo1) not found"); }

        telemetry.update();
    }

    @Override
    public void loop() {
        // ============================================
        // DRIVE LOGIC
        // ============================================
        double yRaw = -gamepad1.left_stick_y;
        double xRaw = gamepad1.left_stick_x;
        double rRaw = gamepad1.right_stick_x;

        double yInput = Math.copySign(yRaw * yRaw, yRaw);
        double xInput = Math.copySign(xRaw * xRaw, xRaw);
        double rInput = Math.copySign(rRaw * rRaw, rRaw);

        if (Math.abs(yRaw) < DEADZONE) yInput = 0;
        if (Math.abs(xRaw) < DEADZONE) xInput = 0;
        if (Math.abs(rRaw) < DEADZONE) rInput = 0;

        boolean slowMode = gamepad1.a;
        double speedMultiplier = slowMode ? SPEED_SLOW : SPEED_NORMAL;

        double flPow = (yInput + xInput + rInput) * speedMultiplier;
        double frPow = (yInput - xInput - rInput) * speedMultiplier;
        double blPow = (yInput - xInput + rInput) * speedMultiplier;
        double brPow = (yInput + xInput - rInput) * speedMultiplier;

        double max = Math.max(1.0, Math.max(Math.abs(flPow), Math.max(Math.abs(frPow), Math.max(Math.abs(blPow), Math.abs(brPow)))));

        if (flOk) fl.setPower(flPow / max);
        if (frOk) fr.setPower(frPow / max);
        if (blOk) bl.setPower(blPow / max);
        if (brOk) br.setPower(brPow / max);

        // ============================================
        // MECHANISMS
        // ============================================

        // --- Launcher (R Bumper) ---
        boolean currentLauncherBtn = gamepad1.right_bumper;
        if (currentLauncherBtn && !lastLauncherBtn) {
            launcherActive = !launcherActive;
        }
        lastLauncherBtn = currentLauncherBtn;

        if (launcherOk) {
            launcher.setPower(launcherActive ? POWER_LAUNCH : 0.0);
        }

        // --- Dual Servo Control (B Button) ---
        // Bボタンひとつで両方のサーボを同時に操作
        boolean currentServoBtn = gamepad1.b;

        if (currentServoBtn && !lastServoBtn) {
            servosActive = !servosActive; // ON/OFF切り替え

            double targetPos = servosActive ? SERVO_POS_ACTIVE : SERVO_POS_REST;

            // 両方のサーボに同じターゲット値を送る
            // (初期化時にDirectionを逆に設定しているので、実際の回転方向は逆になります)
            if (extServoOk) extServo.setPosition(targetPos);
            if (mainServoOk) mainServo.setPosition(targetPos);
        }
        lastServoBtn = currentServoBtn;

        // --- Telemetry ---
        telemetry.addData("Mode", slowMode ? "SLOW" : "NORMAL");
        telemetry.addData("Launcher", launcherActive ? "ON" : "OFF");
        telemetry.addData("Servos", servosActive ? "Active" : "Rest");
        telemetry.update();
    }
}


