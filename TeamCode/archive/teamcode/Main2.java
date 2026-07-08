package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Main2", group = "Linear Opmode")
public class Main2 extends LinearOpMode {

    // --- Definitions ---
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;

    private Servo launcherServo = null;
    private DcMotor plateMotor = null;
    private DcMotor intakeMotor = null;

    @Override
    public void runOpMode() {
        // --- 1. Hardware Mapping ---
        // Ensure these names match your Driver Station Configuration EXACTLY
        leftFront  = hardwareMap.get(DcMotor.class, "left_front");
        leftBack   = hardwareMap.get(DcMotor.class, "left_back");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightBack  = hardwareMap.get(DcMotor.class, "right_back");

        launcherServo = hardwareMap.get(Servo.class, "launcher_servo");
        plateMotor    = hardwareMap.get(DcMotor.class, "plate_motor");
        intakeMotor   = hardwareMap.get(DcMotor.class, "intake_motor");

        // --- 2. Motor Directions ---
        // Reverse left motors so positive power moves the robot forward
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        // --- 3. Zero Power Behavior ---
        // BRAKE: Motors stop immediately (good for arms/lifts)
        // FLOAT: Motors coast to a stop (good for intakes/flywheels)
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        plateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // --- 4. Main Loop ---
        while (opModeIsActive()) {

            // === A. Mecanum Drive Control ===
            double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator ensures we don't exceed power 1.0, keeping proportions correct
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            double frontLeftPower  = (y + x + rx) / denominator;
            double backLeftPower   = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower  = (y + x - rx) / denominator;

            leftFront.setPower(frontLeftPower);
            leftBack.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightBack.setPower(backRightPower);

            // === B. Launcher Servo ===
            if (gamepad1.a) {
                launcherServo.setPosition(1.0);
            } else if (gamepad1.b) {
                launcherServo.setPosition(0.0);
            }

            // === C. Plate Motor (Lift/Arm) ===
            if (gamepad1.dpad_up) {
                plateMotor.setPower(0.5);
            } else if (gamepad1.dpad_down) {
                plateMotor.setPower(-0.5);
            } else {
                plateMotor.setPower(0);
            }

            // === D. Intake Motor ===
            if (gamepad1.right_trigger > 0.1) {
                intakeMotor.setPower(0.8);
            } else if (gamepad1.left_trigger > 0.1) {
                intakeMotor.setPower(-0.8);
            } else {
                intakeMotor.setPower(0);
            }

            // Telemetry
            telemetry.addData("Servo Pos", launcherServo.getPosition());
            telemetry.addData("Plate Power", plateMotor.getPower());
            telemetry.update();
        }
    }
}