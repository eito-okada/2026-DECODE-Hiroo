package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Mecanum + Manual (No Anchor)", group = "Main")
public class main_gen1 extends OpMode {

    // --- ODOMETRY VARIABLES (位置表示用) ---
    private double globalX = 0;
    private double globalY = 0;
    private int lastParPos = 0;
    private int lastPerpPos = 0;
    private static final double TICKS_PER_UNIT = 1000.0;

    private IMU imu;

    // --- DRIVE MOTORS ---
    private DcMotor fl, fr, bl, br;
    private boolean flOk, frOk, blOk, brOk;

    // --- MECHANISM MOTORS ---
    private DcMotor intake, arm, launcher1, launcher2;
    private boolean intakeOk, armOk, launcher1Ok, launcher2Ok;

    // --- TOGGLE VARIABLES ---
    private boolean launcherActive = false;
    private boolean lastLauncherBtn = false;

    // --- SETTINGS ---
    private static final double DEADZONE = 0.05;
    private static final double POWER_INTAKE = 1.0;

    // アーム出力 MAX
    private static final double POWER_ARM = 1.0;

    private static final double POWER_LAUNCH = 1.0;
    private static final double TRIGGER_THRESHOLD = 0.5;

    @Override
    public void init() {
        // 1. IMU Init
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        imu.resetYaw();

        // 2. Drive Motors Init
        try {
            fl = hardwareMap.get(DcMotor.class, "motor0");
            fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            fl.setDirection(DcMotorSimple.Direction.REVERSE);
            fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            flOk = true;
        } catch (Exception e) {}

        try {
            bl = hardwareMap.get(DcMotor.class, "motor1");
            bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bl.setDirection(DcMotorSimple.Direction.REVERSE);
            bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            blOk = true;
        } catch (Exception e) {}

        try {
            fr = hardwareMap.get(DcMotor.class, "motor2");
            fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            fr.setDirection(DcMotorSimple.Direction.FORWARD);
            fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frOk = true;
        } catch (Exception e) {}

        try {
            br = hardwareMap.get(DcMotor.class, "motor3");
            br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            br.setDirection(DcMotorSimple.Direction.FORWARD);
            br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            brOk = true;
        } catch (Exception e) {}

        // 3. Mechanism Motors Init
        try { intake = hardwareMap.get(DcMotor.class, "motor4"); intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); intakeOk = true; } catch (Exception e) {}
        try { arm = hardwareMap.get(DcMotor.class, "motor5"); arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); arm.setDirection(DcMotorSimple.Direction.REVERSE); arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); armOk = true; } catch (Exception e) {}
        try { launcher1 = hardwareMap.get(DcMotor.class, "motor6"); launcher1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); launcher1.setDirection(DcMotorSimple.Direction.FORWARD); launcher1Ok = true; } catch (Exception e) {}
        try { launcher2 = hardwareMap.get(DcMotor.class, "motor7"); launcher2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); launcher2.setDirection(DcMotorSimple.Direction.REVERSE); launcher2Ok = true; } catch (Exception e) {}

        telemetry.update();
    }

    @Override
    public void loop() {
        // ============================================
        // 1. ODOMETRY UPDATE (座標計算のみ実施・制御には使いません)
        // ============================================
        int currentParPos = flOk ? fl.getCurrentPosition() : 0;
        int currentPerpPos = blOk ? bl.getCurrentPosition() : 0;
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        int dPar = currentParPos - lastParPos;
        int dPerp = currentPerpPos - lastPerpPos;
        lastParPos = currentParPos;
        lastPerpPos = currentPerpPos;

        double deltaX = (dPar * Math.sin(heading) + dPerp * Math.cos(heading));
        double deltaY = (dPar * Math.cos(heading) - dPerp * Math.sin(heading));

        globalX += deltaX / TICKS_PER_UNIT;
        globalY += deltaY / TICKS_PER_UNIT;

        // ============================================
        // 2. DRIVE LOGIC (完全マニュアル操作)
        // ============================================

        double yInput = -gamepad1.left_stick_y; // 前後
        double xInput = gamepad1.left_stick_x;  // 左右
        double rInput = gamepad1.right_stick_x; // 回転

        // デッドゾーン処理
        if (Math.abs(yInput) < DEADZONE) yInput = 0;
        if (Math.abs(xInput) < DEADZONE) xInput = 0;
        if (Math.abs(rInput) < DEADZONE) rInput = 0;

        // メカナムホイール出力計算
        double flPow = yInput + xInput + rInput;
        double frPow = yInput - xInput - rInput;
        double blPow = yInput - xInput + rInput;
        double brPow = yInput + xInput - rInput;

        // パワーの正規化 (最大値が1を超えないようにする)
        double max = Math.max(1.0, Math.max(Math.abs(flPow), Math.max(Math.abs(frPow), Math.max(Math.abs(blPow), Math.abs(brPow)))));

        if (flOk) fl.setPower(flPow / max);
        if (frOk) fr.setPower(frPow / max);
        if (blOk) bl.setPower(blPow / max);
        if (brOk) br.setPower(brPow / max);


        // ============================================
        // 3. MECHANISMS CONTROL
        // ============================================

        boolean inputReverse = gamepad1.left_bumper;
        double dirMultiplier = inputReverse ? -1.0 : 1.0;

        // Intake
        if (intakeOk) {
            if (gamepad1.left_trigger > TRIGGER_THRESHOLD) intake.setPower(POWER_INTAKE * dirMultiplier);
            else intake.setPower(0);
        }

        // Arm (Extension) - MAX Power
        if (armOk) {
            if (gamepad1.right_trigger > TRIGGER_THRESHOLD) arm.setPower(POWER_ARM * dirMultiplier);
            else arm.setPower(0);
        }

        // Launcher
        boolean currentLauncherBtn = gamepad1.right_bumper;
        if (currentLauncherBtn && !lastLauncherBtn) launcherActive = !launcherActive;
        lastLauncherBtn = currentLauncherBtn;

        double launchPower = launcherActive ? POWER_LAUNCH : 0.0;
        if (launcher1Ok) launcher1.setPower(launchPower);
        if (launcher2Ok) launcher2.setPower(launchPower);

        // --- Telemetry ---
        telemetry.addData("Mode", "Manual Drive (No Anchor)");
        telemetry.addData("Arm Power", "%.1f", armOk ? arm.getPower() : 0);
        telemetry.addData("Launcher", launcherActive ? "ON" : "OFF");
        telemetry.addData("Position", "X:%.1f, Y:%.1f", globalX, globalY);
        telemetry.update();
    }
}