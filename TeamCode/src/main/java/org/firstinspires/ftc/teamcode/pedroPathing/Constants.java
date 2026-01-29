package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple; // ★ここを確認
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

    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardEncoderDirection("motor0")   // ✅ this replaces parallelEncoderName
            .strafeEncoderName("motor3")
            .forwardEncoderDirection(DcMotorSimple.Direction.FORWARD)
            .strafeEncoderDirection(DcMotorSimple.Direction.FORWARD)
            .imuName("imu")
            .imuLogoDirection(RevHubOrientationOnRobot.LogoFacingDirection.UP)
            .imuUsbDirection(RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)
            .forwardPodY(5.5)
            .strafePodX(-2.5)
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