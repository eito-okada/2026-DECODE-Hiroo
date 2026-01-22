package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Simple Launcher")
public class Main extends LinearOpMode {

    // Declare the servo
    private Servo launcherServo;

    @Override
    public void runOpMode() {
        // Initialize the servo - name must match your config exactly
        launcherServo = hardwareMap.get(Servo.class, "launcher_servo");

        // Set starting position (0 degrees)
        launcherServo.setPosition(0.0);

        waitForStart();

        while (opModeIsActive()) {
            // When button A is pressed:
            if (gamepad2.b) {
                // 1. Move to 90 degrees immediately
                launcherServo.setPosition(0.5);

                // 2. Tiny pause (0.15s) so the motor has time to physically reach 90
                sleep(150);

                // 3. Move back to 0 degrees immediately
                launcherServo.setPosition(0.0);
                telemetry.addData("Circle Pressed", gamepad2.b);
            }

            telemetry.addData("Servo Position", launcherServo.getPosition());
            telemetry.update();
        }
    }
}