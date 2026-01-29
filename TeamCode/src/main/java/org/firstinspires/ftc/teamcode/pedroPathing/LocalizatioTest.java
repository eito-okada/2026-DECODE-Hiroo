package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Localization Test (No Pinpoint)", group = "Debug")
public class LocalizatioTest extends OpMode {

    private Follower follower;

    @Override
    public void init() {
        // Create the follower using the NEW Constants (TwoWheelLocalizer)
        try {
            follower = Constants.createFollower(hardwareMap);
            telemetry.addLine("Follower created successfully.");
        } catch (Exception e) {
            telemetry.addData("Error", "Could not create follower: " + e.getMessage());
        }

        telemetry.addLine("Press PLAY to start tracking.");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (follower != null) {
            follower.update();

            com.pedropathing.geometry.Pose pose = follower.getPose();

            telemetry.addData("X (Inches)", pose.getX());
            telemetry.addData("Y (Inches)", pose.getY());
            telemetry.addData("Heading (Deg)", Math.toDegrees(pose.getHeading()));

            telemetry.addLine("\n--- Raw Encoders ---");

        }

        telemetry.update();
    }
}