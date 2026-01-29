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
        follower.update();

        // Get the pose (Position + Heading)
        com.pedropathing.geometry.Pose pose = follower.getPose();

        // X and Y are in Inches
        telemetry.addData("X (Inches)", pose.getX());
        telemetry.addData("Y (Inches)", pose.getY());

        // Heading is in Radians -> Convert to Degrees for humans
        double headingRadians = pose.getHeading();
        double headingDegrees = Math.toDegrees(headingRadians);

        telemetry.addData("Heading (Radians)", "%.2f", headingRadians); // Shows 3.14
        telemetry.addData("Heading (Degrees)", "%.1f", headingDegrees); // Shows 180.0

        telemetry.update();
    }

}