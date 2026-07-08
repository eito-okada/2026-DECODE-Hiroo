package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        initAprilTag();

        telemetry.addLine("Webcam ready. Tracking tag ID " + TARGET_TAG_ID);
        telemetry.addLine("Press START to begin.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            showTargetTag();
            telemetry.update();
            sleep(20); // share the CPU
        }

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

    /** Finds the target tag and displays its heading (deg) and distance (cm). */
    private void showTargetTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        AprilTagDetection target = null;
        for (AprilTagDetection detection : detections) {
            if (detection.id == TARGET_TAG_ID) {
                target = detection;
                break;
            }
        }

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
}
