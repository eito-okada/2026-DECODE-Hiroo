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
        // ★これが無いとPoseが更新されないことが多い
        follower.update();

        // Pose表示（PedroPathing側が更新している値）
        com.pedropathing.geometry.Pose pose = follower.getPose();

        telemetry.addData("x", pose.getX());
        telemetry.addData("y", pose.getY());
        telemetry.addData("heading", pose.getHeading());

        // Pinpointが見つかっているかも常に出す（0固定ならまずここを疑う）
        telemetry.addData("pinpointOk", pinpointOk);

        telemetry.update();
    }
}
