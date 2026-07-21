package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static final FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.3)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.08, 0, 0.005, 0.01))
            .headingPIDFCoefficients(new PIDFCoefficients(1.1, 0, 0.05, 0.025))
            .forwardZeroPowerAcceleration(-45.265)
            .lateralZeroPowerAcceleration(-67.928)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025,0.0,0.00001,0.6,0.01))
            .centripetalScaling(0.0001);
    public static final MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .leftFrontMotorName("motor1")
            .leftRearMotorName("motor2")
            .rightFrontMotorName("motor4")
            .rightRearMotorName("motor5")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(73.541)
            .yVelocity(56.1139);

    // goBILDA Pinpoint localization (2 pods + built-in IMU).
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(11.5)
            .strafePodX(7.8)
            .distanceUnit(DistanceUnit.CM)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();

    }
}
