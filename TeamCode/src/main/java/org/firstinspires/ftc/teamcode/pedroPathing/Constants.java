package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(7);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("motor0")
            .rightRearMotorName("motor1")
            .leftRearMotorName("motor3")
            .leftFrontMotorName("motor2")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    // Fixed TwoWheelConstants based on your provided method names
    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            // 1. Hardware Map Names (The ones you found)
            .forwardEncoder_HardwareMapName("motor0")
            .strafeEncoder_HardwareMapName("motor3")

            // 2. Encoder Directions (DcMotorSimple.Direction)
            // Run LocalizationTest: If X goes down when you push forward, flip REVERSE to FORWARD
            .forwardEncoderDirection(1.0)
            .strafeEncoderDirection(-1.0)

            .IMU_HardwareMapName("imu")

            // ★ これを試してください（2行を1行にまとめる）
            .IMU_Orientation(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            ))

            // 4. Physical Pod Offsets (インチ単位)
            .forwardPodY(5.5)   // 左ならプラス
            .strafePodX(-2.5)   // 前ならプラス

            // 5. Tuning Values
            .forwardTicksToInches(0.0005)
            .strafeTicksToInches(0.0005);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .twoWheelLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}