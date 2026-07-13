package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "AprilTag Vision Auto", group = "Main")
public class AprilTagAuto extends LinearOpMode {

    // The AprilTag ID we care about. Set this to whichever DECODE tag you want to track.
    private static final int TARGET_TAG_ID = 20;

    // Must match the webcam name in your Driver Station robot configuration.
    private static final String WEBCAM_NAME = "Webcam 1";

    // Must match the pan motor's name in your Driver Station robot configuration.
    private static final String PAN_MOTOR_NAME = "motor5";

    // Proportional gain: motor power per degree of bearing error.
    private static final double PAN_KP = 0.03;
    private static final double PAN_MAX_POWER = 0.55;
    // Stop correcting once the tag is within this many degrees of centered, to avoid jitter.
    private static final double PAN_DEADBAND_DEG = 2.0;

    // Soft limits (encoder ticks from the position the motor was at when the op mode started)
    // so the webcam can't wind up its own wiring by spinning past its mount's range of motion.
    private static final int PAN_MIN_TICKS = -400;
    private static final int PAN_MAX_TICKS = 400;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private DcMotor panMotor;

    @Override
    public void runOpMode() {

        initAprilTag();
        initPanMotor();

        telemetry.addLine("Webcam ready. Tracking tag ID " + TARGET_TAG_ID);
        telemetry.addLine("Press START to begin.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            AprilTagDetection target = findTargetTag();
            showTargetTag(target);
            trackTagWithPanMotor(target);
            telemetry.update();
            sleep(20); // share the CPU
        }

        panMotor.setPower(0);
        visionPortal.close();
    }

    /** Builds the AprilTag processor (cm output) and the webcam vision portal. */
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                // Report distance in centimeters and angles in degrees.
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                .addProcessor(aprilTag)
                .build();
    }

    /** Sets up the geared DC motor that pans the webcam mount. */
    private void initPanMotor() {
        panMotor = hardwareMap.get(DcMotor.class, PAN_MOTOR_NAME);
        panMotor.setDirection(DcMotor.Direction.FORWARD);
        panMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        panMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        panMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /** Finds the detection matching TARGET_TAG_ID, or null if it isn't currently visible. */
    private AprilTagDetection findTargetTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.id == TARGET_TAG_ID) {
                return detection;
            }
        }
        return null;
    }

    /** Displays the target tag's heading (deg) and distance (cm). */
    private void showTargetTag(AprilTagDetection target) {
        if (target != null && target.ftcPose != null) {
            // bearing = angle to turn to face the tag (our "heading")
            // range   = straight-line distance to the tag
            telemetry.addData("Tag " + TARGET_TAG_ID, "VISIBLE");
            telemetry.addData("Heading (deg)", "%.1f", target.ftcPose.bearing);
            telemetry.addData("Distance (cm)", "%.1f", target.ftcPose.range);
        } else {
            telemetry.addData("Tag " + TARGET_TAG_ID, "not seen");
        }
    }

    /**
     * Drives the pan motor to center the target tag in the webcam's view, using the tag's
     * bearing as a proportional error signal. Holds still if the tag isn't visible so the
     * webcam keeps looking wherever it last saw it.
     */
    private void trackTagWithPanMotor(AprilTagDetection target) {
        int currentTicks = panMotor.getCurrentPosition();

        if (target == null || target.ftcPose == null) {
            panMotor.setPower(0);
            telemetry.addData("Pan", "holding (no tag)");
            return;
        }

        double error = target.ftcPose.bearing;
        double power = 0;

        if (Math.abs(error) >= PAN_DEADBAND_DEG) {
            power = -error * PAN_KP;
            power = Math.max(-PAN_MAX_POWER, Math.min(PAN_MAX_POWER, power));

            // Don't drive further past the soft limits.
            if (currentTicks <= PAN_MIN_TICKS && power < 0) power = 0;
            if (currentTicks >= PAN_MAX_TICKS && power > 0) power = 0;
        }

        panMotor.setPower(power);
        telemetry.addData("Pan power", "%.2f", power);
        telemetry.addData("Pan ticks", currentTicks);
    }
}
