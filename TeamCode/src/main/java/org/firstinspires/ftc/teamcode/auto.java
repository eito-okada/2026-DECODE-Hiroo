package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Configurable Auto (Gamepad Tune)", group = "Backup")
public class auto extends LinearOpMode {

    private double driveSpeed = 0.50; // 50%
    private double driveTime  = 2.0;  // 2.0 Seconds

    boolean lastUp = false, lastDown = false;
    boolean lastRight = false, lastLeft = false;

    @Override
    public void runOpMode() {

        DcMotor fl = hardwareMap.get(DcMotor.class, "motor2");
        DcMotor fr = hardwareMap.get(DcMotor.class, "motor0");
        DcMotor bl = hardwareMap.get(DcMotor.class, "motor3");
        DcMotor br = hardwareMap.get(DcMotor.class, "motor1");
        DcMotor shooter = hardwareMap.get(DcMotor.class, "motor7");
        DcMotor transfer = hardwareMap.get(DcMotor.class, "motor6");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.addLine("--- CONFIG MODE ---");
        telemetry.addLine("Use D-Pad to adjust settings.");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {

            // --- SPEED CONTROL (Up/Down) ---
            if (gamepad1.dpad_up && !lastUp) {
                driveSpeed += 0.01; // +1%
            }
            if (gamepad1.dpad_down && !lastDown) {
                driveSpeed -= 0.01; // -1%
            }

            if (gamepad1.dpad_right && !lastRight) {
                driveTime += 0.1; // +0.1 sec
            }
            if (gamepad1.dpad_left && !lastLeft) {
                driveTime -= 0.1; // -0.1 sec
            }

            if (driveSpeed < 0) driveSpeed = 0;
            if (driveSpeed > 1) driveSpeed = 1;
            if (driveTime < 0)  driveTime = 0;

            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            lastRight = gamepad1.dpad_right;
            lastLeft = gamepad1.dpad_left;

            telemetry.addLine(">> READY TO START <<");
            telemetry.addLine("--------------------");
            telemetry.addData("Speed (Up/Dn)", "%.0f %%", driveSpeed * 100);
            telemetry.addData("Time  (Rt/Lt)", "%.1f sec", driveTime);
            telemetry.addLine("--------------------");
            telemetry.update();
        }



        if (opModeIsActive()) {

            telemetry.addData("Action", "Driving for %.1fs at %.0f%%", driveTime, driveSpeed*100);
            telemetry.update();

            fl.setPower(driveSpeed);
            fr.setPower(driveSpeed);
            bl.setPower(driveSpeed);
            br.setPower(driveSpeed);

            sleep((long)(driveTime * 1000));

            // Stop
            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);

            sleep(500);

            telemetry.addData("Action", "Shooting...");
            telemetry.update();

            shooter.setPower(1.0);
            sleep(2000); // Spin up

            transfer.setPower(1.0);
            sleep(2000); // Feed

            // Stop All
            shooter.setPower(0);
            transfer.setPower(0);
        }
    }
}