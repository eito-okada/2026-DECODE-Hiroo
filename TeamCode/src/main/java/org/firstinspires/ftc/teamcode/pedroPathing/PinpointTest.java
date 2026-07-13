package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * TEST 0 — Is the Pinpoint alive? (No Pedro involved.)
 *
 * This talks to the goBILDA Pinpoint driver directly, so it only checks one thing:
 * is the module wired correctly and configured on the Driver Station as "pinpoint"?
 *
 * HOW TO PASS:
 *   1. Deploy, run this OpMode, press PLAY.
 *   2. Status should read READY (not FAULT / NOT_READY).
 *   3. Push the robot FORWARD  -> X (mm) changes.
 *   4. Push the robot SIDEWAYS -> Y (mm) changes.
 *   5. Rotate the robot        -> Heading (deg) changes.
 *
 * Direction correctness does NOT matter yet — we only care that the numbers MOVE.
 * If nothing moves or Status is bad, it's a wiring/config problem, not a code problem.
 */
@TeleOp(name = "Test 0 - Pinpoint Alive", group = "Pedro Tests")
public class PinpointTest extends OpMode {

    private GoBildaPinpointDriver pinpoint;

    @Override
    public void init() {
        // Name must match the I2C device you added in the Robot Configuration.
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Tell the module which pods you have. Change to goBILDA_SWINGARM_POD if that's your pod.
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // Zero everything so we start from 0/0/0.
        pinpoint.resetPosAndIMU();

        telemetry.addLine("Pinpoint found. Press PLAY, then push/rotate the robot by hand.");
        telemetry.update();
    }

    @Override
    public void loop() {
        pinpoint.update(); // read the module every loop

        telemetry.addData("Status", pinpoint.getDeviceStatus());
        telemetry.addData("Refresh rate (Hz)", "%.0f", pinpoint.getFrequency());
        telemetry.addData("X (mm)", "%.1f", pinpoint.getPosX(DistanceUnit.MM));
        telemetry.addData("Y (mm)", "%.1f", pinpoint.getPosY(DistanceUnit.MM));
        telemetry.addData("Heading (deg)", "%.1f", pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.update();
    }
}
