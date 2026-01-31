package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@Autonomous(name = "紅の機械 V3", group = "Main")
public class auto3 extends LinearOpMode {

    // --- HARDWARE ---
    private DcMotorEx fl, fr, bl, br;
    private DcMotorEx parPod, perpPod; // The two dead wheels
    private IMU imu;

    private DcMotor gecko;

    // --- TUNING (CRITICAL) ---
    // 1. TICKS_PER_INCH: Push robot 24 inches. If it says 12, double this number.
    final double PAR_TICKS_PER_INCH = 2000.0;
    final double PERP_TICKS_PER_INCH = 2000.0;

    // 2. OFFSETS (Inches from center of robot)
    // PAR_OFFSET: Distance from center to Parallel Pod. (Positive if LEFT of center)
    final double PAR_OFFSET = 6.0;
    // PERP_OFFSET: Distance from center to Perpendicular Pod. (Positive if FORWARD of center)
    final double PERP_OFFSET = -4.5;

    // --- STATE VARIABLES ---
    double myX = 0, myY = 0, myHeading = 0;
    int lastParPos = 0, lastPerpPos = 0;
    double lastImuHeading = 0;

    // --- PID SETTINGS (Drive Power) ---
    double driveKp = 0.06; // Power per inch of error
    double turnKp = 1.0;   // Power per radian of error
    double xyTolerance = 1.5; // Allow 1.5 inch error
    double headTolerance = Math.toRadians(2.0); // Allow 2 degree error

    @Override
    public void runOpMode() {

        // 1. INIT MOTORS
        fl = hardwareMap.get(DcMotorEx.class, "motor0");
        fr = hardwareMap.get(DcMotorEx.class, "motor1");
        bl = hardwareMap.get(DcMotorEx.class, "motor2");
        br = hardwareMap.get(DcMotorEx.class, "motor3");

        // 2. INIT PODS (Assume Port 0 and 3 on encoder ports, adjust as needed)
        parPod = hardwareMap.get(DcMotorEx.class, "motor0"); // Parallel Pod
        perpPod = hardwareMap.get(DcMotorEx.class, "motor3"); // Perpendicular Pod

        DcMotor intake = hardwareMap.get(DcMotor.class, "motor4");
        gecko = hardwareMap.get(DcMotor.class, "motor6");
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "motor7");

        // 3. INIT IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        // 4. RESET & CONFIG
        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        // Reset Encoders
        parPod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        perpPod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        parPod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        perpPod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu.resetYaw();

        telemetry.addLine("Ready. Push robot to test tracking.");
        telemetry.addLine("X should change when moving Side-to-Side.");
        telemetry.addLine("Y should change when moving Forward/Back.");
        telemetry.update();

        // WAIT LOOP (Show position while waiting)
        while(!isStarted() && !isStopRequested()) {
            updateOdometry();
            telemetry.addData("X", "%.1f", myX);
            telemetry.addData("Y", "%.1f", myY);
            telemetry.addData("H", "%.1f", Math.toDegrees(myHeading));
            telemetry.update();
        }

        // --- PATH EXECUTION ---
        if (opModeIsActive()) {

            // 1. Shoot Preload
            shooter.setPower(0.6);
            driveTo(0, 0, 0, 1.0); // Hold position
            fire();

            // 2. Drive to Ball Stack (Coordinate: X=24, Y=12)
            // Turn intake ON early
            intake.setPower(1.0);
            driveTo(24, 12, Math.toRadians(-90), 4.0); // 4 sec timeout

            // 3. Drive back to Shoot (Coordinate: X=4, Y=0)
            intake.setPower(0);
            driveTo(4, 0, Math.toRadians(0), 4.0);
            fire();

            // 4. Park
            driveTo(0, 24, 0, 3.0);
        }
    }

    // --- ACTIONS ---

    private void fire() {
        gecko.setPower(1.0);
        sleep(800);
        gecko.setPower(0);
    }

    // --- CORE DRIVE LOGIC ---

    private void driveTo(double targetX, double targetY, double targetHeading, double timeoutS) {
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && timer.seconds() < timeoutS) {
            updateOdometry();

            // Calculate Error (Distance to target)
            double errorX = targetX - myX;
            double errorY = targetY - myY;
            double errorH = angleWrap(targetHeading - myHeading);

            // Distance Check
            if (Math.abs(errorX) < xyTolerance && Math.abs(errorY) < xyTolerance && Math.abs(errorH) < headTolerance) {
                break; // We are there!
            }

            // Field Centric -> Robot Centric Conversion
            double sin = Math.sin(-myHeading);
            double cos = Math.cos(-myHeading);

            double robotX = errorX * cos - errorY * sin;
            double robotY = errorX * sin + errorY * cos;

            // Apply PID Power
            // Note: X in robot centric is Forward for some math, but Strafe for others.
            // For standard Mecanum: Y is Forward, X is Strafe.
            // But our rotation math above puts Forward in X. Let's swap to standard FTC Frame:
            // Standard FTC: X is Forward, Y is Left.
            // Let's stick to: xDest is forward speed, yDest is strafe speed.

            double driveSpeed = robotX * driveKp;
            double strafeSpeed = robotY * driveKp;
            double turnSpeed = -errorH * turnKp;

            // Clip speeds
            driveSpeed = clip(driveSpeed);
            strafeSpeed = clip(strafeSpeed);
            turnSpeed = clip(turnSpeed);

            // Set Power
            // FL = Drive + Strafe + Turn
            double flP = driveSpeed - strafeSpeed - turnSpeed;
            double blP = driveSpeed + strafeSpeed - turnSpeed;
            double frP = driveSpeed + strafeSpeed + turnSpeed;
            double brP = driveSpeed - strafeSpeed + turnSpeed;

            // Normalize
            double max = Math.max(Math.abs(flP), Math.max(Math.abs(blP), Math.max(Math.abs(frP), Math.abs(brP))));
            if (max > 1.0) { flP/=max; blP/=max; frP/=max; brP/=max; }

            fl.setPower(flP); bl.setPower(blP); fr.setPower(frP); br.setPower(brP);

            telemetry.addData("Target", "%.1f, %.1f", targetX, targetY);
            telemetry.addData("Current", "%.1f, %.1f", myX, myY);
            telemetry.update();
        }
        stopMotors();
    }

    // --- ODOMETRY UPDATE (THE MATH) ---
    private void updateOdometry() {
        // 1. Read Sensors
        int currentPar = parPod.getCurrentPosition();
        int currentPerp = perpPod.getCurrentPosition();
        double currentImu = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // 2. Calculate Deltas (Change since last loop)
        int dnPar = currentPar - lastParPos;
        int dnPerp = currentPerp - lastPerpPos;
        double dnHeading = angleWrap(currentImu - lastImuHeading);

        // 3. Save for next loop
        lastParPos = currentPar;
        lastPerpPos = currentPerp;
        lastImuHeading = currentImu;

        // 4. Convert Ticks to Inches
        double dPar = dnPar / PAR_TICKS_PER_INCH;
        double dPerp = dnPerp / PERP_TICKS_PER_INCH;

        // 5. Remove "Turn" component (The Arc)
        // If we just spin in place, the pods still roll. We must subtract that.
        // Parallel Pod moves: offset * radians
        double dForward = dPar - (PAR_OFFSET * dnHeading);
        double dStrafe = dPerp - (PERP_OFFSET * dnHeading);

        // 6. Rotate these robot-centric moves to Global Field Coordinates
        // This calculates how much X and Y changed on the actual field
        double avgHeading = myHeading + (dnHeading / 2.0); // Small optimization

        // Standard Rotation Matrix
        double globalX = dForward * Math.cos(avgHeading) - dStrafe * Math.sin(avgHeading);
        double globalY = dForward * Math.sin(avgHeading) + dStrafe * Math.cos(avgHeading);

        // 7. Update Global Position
        myX += globalX;
        myY += globalY;
        myHeading = currentImu;
    }

    // --- UTILS ---

    private double angleWrap(double radians) {
        while (radians > Math.PI) radians -= 2 * Math.PI;
        while (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }

    private double clip(double val) {
        return Math.max(-1.0, Math.min(1.0, val));
    }

    private void stopMotors() {
        fl.setPower(0); fr.setPower(0); bl.setPower(0); br.setPower(0);
    }
}