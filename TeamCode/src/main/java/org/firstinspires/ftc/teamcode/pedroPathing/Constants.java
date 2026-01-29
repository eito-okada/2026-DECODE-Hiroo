package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants; // Changed to TwoWheel
import com.pedropathing.localization.Encoder;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot; // Required for IMU
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

    // FIX: Using TwoWheelConstants for 2 pods + IMU
    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            // 1. HARDWARE NAMES (From Driver Station Config)
            .forwardEncoderName("motor0") // The parallel pod port
            .strafeEncoderName("motor3")  // The perpendicular pod port
            .imuName("imu")               // The built-in IMU name

            // 2. DIRECTIONS (Run LocalizationTest to check)
            .forwardEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.FORWARD)

            // 3. IMU ORIENTATION (Crucial for 2-wheel!)
            // Check how your Control Hub is mounted on the robot
            .imuLogoDirection(RevHubOrientationOnRobot.LogoFacingDirection.UP)
            .imuUsbDirection(RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)

            // 4. PHYSICAL OFFSETS (Inches from center of robot)
            // forwardPodY: Distance Left(+) or Right(-) of center
            .forwardPodY(5.5)
            // strafePodX: Distance Forward(+) or Backward(-) of center
            .strafePodX(-2.5)

            // 5. TUNING VALUES (Run ForwardTuner and StrafeTuner)
            .forwardTicksToInches(0.0005)
            .strafeTicksToInches(0.0005);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .twoWheelLocalizer(localizerConstants) // FIX: Changed to twoWheelLocalizer
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}