package org.firstinspires.ftc.teamcode.pedroPathing;

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

    private static final DistanceUnit PINPOINT_UNIT = DistanceUnit.INCH;
    private static final String PINPOINT_NAME = "pinpoint";

    private static final double FORWARD_POD_Y_IN = -5.0;
    private static final double STRAFE_POD_X_IN  =  0.5;

    private static final GoBildaPinpointDriver.EncoderDirection FORWARD_DIR =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    private static final GoBildaPinpointDriver.EncoderDirection STRAFE_DIR  =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    private static final GoBildaPinpointDriver.GoBildaOdometryPods POD_TYPE =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

    public static final PinpointConstants localizerConstants = new PinpointConstants()
            .distanceUnit(PINPOINT_UNIT)
            .hardwareMapName(PINPOINT_NAME)
            .forwardPodY(FORWARD_POD_Y_IN)
            .strafePodX(STRAFE_POD_X_IN)
            .encoderResolution(POD_TYPE)
            .forwardEncoderDirection(FORWARD_DIR)
            .strafeEncoderDirection(STRAFE_DIR);


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}