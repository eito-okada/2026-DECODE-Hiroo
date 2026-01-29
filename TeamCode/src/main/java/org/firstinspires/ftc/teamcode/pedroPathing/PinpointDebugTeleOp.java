package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Pinpoint Debug (Pose Check)", group = "Debug")
public class PinpointDebugTeleOp extends OpMode {

    private Follower follower;
    private GoBildaPinpointDriver pinpoint;
    private boolean pinpointOk = false;

    @Override
    public void init() {
        // 1) Pinpoint が hardwareMap で取れるか確認
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            pinpointOk = (pinpoint != null);
        } catch (Exception e) {
            pinpointOk = false;
        }

        // 2) Follower作成（Pinpoint localizer 使用）
        follower = Constants.createFollower(hardwareMap);

        telemetry.addLine("=== Pinpoint Debug ===");
        telemetry.addData("Pinpoint name in config", "pinpoint");
        telemetry.addData("Pinpoint hardwareMap OK?", pinpointOk);

        // HardwareDevice由来の情報（取れれば表示）
        if (pinpointOk) {
            telemetry.addData("Device", pinpoint.getDeviceName());
            telemetry.addData("Conn", pinpoint.getConnectionInfo());
            telemetry.addData("Version", pinpoint.getVersion());
        } else {
            telemetry.addLine("!! Pinpoint not found. Check RC Config name exactly.");
        }

        telemetry.addLine("Press PLAY, then move robot by hand to see x,y change.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // 1. Update Pedro Pathing
        follower.update();

        // 2. FORCE update the raw Pinpoint driver to check hardware directly
        if (pinpointOk) {
            pinpoint.update(); // This ensures we get fresh data from the device

            // Check if the device is actually ready
            telemetry.addData("Device Status", pinpoint.getDeviceStatus());

            // Check raw encoder positions (if these stay 0, your pods are unplugged)
            // Get the raw position object first
            org.firstinspires.ftc.robotcore.external.navigation.Pose2D rawPose = pinpoint.getPosition();

// Now get X and Y from that object, specifying the unit (INCH)
            telemetry.addData("Raw X (in)", rawPose.getX(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH));
            telemetry.addData("Raw Y (in)", rawPose.getY(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH));
        }

        // 3. Get Pedro's interpreted Pose
        com.pedropathing.geometry.Pose pose = follower.getPose();

        telemetry.addData("--- Pedro Pose ---", "");
        telemetry.addData("x", pose.getX());
        telemetry.addData("y", pose.getY());
        telemetry.addData("heading", pose.getHeading());

        telemetry.addData("pinpointOk", pinpointOk);
        telemetry.update();
    }
}
