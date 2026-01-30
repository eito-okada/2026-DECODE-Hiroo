package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Totally Auto V4", group = "Main")
public class auto extends LinearOpMode {

    // --- HARDWARE ---
    private DcMotor fl, fr, bl, br;
    private DcMotor shooter, transfer, intake;

    // --- STEPS DEFINITION ---
    private enum ActionType { FORWARD, BACKWARD, LEFT, RIGHT, TURN_LEFT, TURN_RIGHT, SHOOT }

    private static class AutoStep {
        ActionType type;
        double power;
        double seconds;
        double delay;

        public AutoStep(ActionType t, double p, double s, double d) {
            this.type = t;
            this.power = p;
            this.seconds = s;
            this.delay = d;
        }

        @NonNull
        @Override
        public String toString() {
            if (type == ActionType.SHOOT) {
                return String.format("SHOOT %.0f%% (Wait %.1fs)", power * 100, delay);
            }
            return String.format("%s (%.0f%%) for %.1fs", type, power * 100, seconds);
        }
    }

    private List<AutoStep> program = new ArrayList<>();

    @Override
    public void runOpMode() {

        // 1. HARDWARE MAPPING
        // CHECK YOUR CONFIG: Is motor0 actually Front Left?
        fl = hardwareMap.get(DcMotor.class, "motor0");
        bl = hardwareMap.get(DcMotor.class, "motor1");
        fr = hardwareMap.get(DcMotor.class, "motor2");
        br = hardwareMap.get(DcMotor.class, "motor3");

        intake   = hardwareMap.get(DcMotor.class, "motor4");
        transfer = hardwareMap.get(DcMotor.class, "motor6");
        shooter  = hardwareMap.get(DcMotor.class, "motor7");

        // 2. MOTOR DIRECTIONS (CRITICAL)
        // If your robot spins when told to go forward, toggle these REVERSE/FORWARD values.
        // Standard Mecanum: Left side REVERSE, Right side FORWARD.
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        // 3. ZERO POWER BEHAVIOR
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // --- MENU VARIABLES ---
        int selectedRow = 0;
        ActionType curType = ActionType.FORWARD;
        double curPower = 0.5;
        double curTime = 1.0;
        double curDelay = 1.0;

        boolean lastUp = false, lastDown = false, lastLeft = false, lastRight = false;
        boolean lastA = false, lastB = false;

        ActionType[] types = ActionType.values();

        // ==========================================
        // CREATOR MODE (INIT PHASE)
        // ==========================================
        while (!isStarted() && !isStopRequested()) {

            // --- MENU NAVIGATION ---
            if (gamepad1.dpad_up && !lastUp) selectedRow--;
            if (gamepad1.dpad_down && !lastDown) selectedRow++;
            if (selectedRow < 0) selectedRow = 3;
            if (selectedRow > 3) selectedRow = 0;

            // --- VALUE CHANGE ---
            if (gamepad1.dpad_right && !lastRight) {
                if (selectedRow == 0) curType = types[(curType.ordinal() + 1) % types.length];
                else if (selectedRow == 1) curPower = Math.min(1.0, curPower + 0.1);
                else if (selectedRow == 2) curTime += 0.1;
                else curDelay += 0.1;
            }

            if (gamepad1.dpad_left && !lastLeft) {
                if (selectedRow == 0) {
                    int prev = curType.ordinal() - 1;
                    if (prev < 0) prev = types.length - 1;
                    curType = types[prev];
                } else if (selectedRow == 1) curPower = Math.max(0.1, curPower - 0.1);
                else if (selectedRow == 2) curTime = Math.max(0.1, curTime - 0.1);
                else curDelay = Math.max(0.0, curDelay - 0.1);
            }

            // --- ADD / DELETE STEPS ---
            if (gamepad1.a && !lastA) program.add(new AutoStep(curType, curPower, curTime, curDelay));
            if (gamepad1.b && !lastB && !program.isEmpty()) program.remove(program.size() - 1);

            // --- UPDATE TRACKERS ---
            lastUp = gamepad1.dpad_up; lastDown = gamepad1.dpad_down;
            lastLeft = gamepad1.dpad_left; lastRight = gamepad1.dpad_right;
            lastA = gamepad1.a; lastB = gamepad1.b;

            // --- TELEMETRY ---
            telemetry.addLine("=== CREATOR MODE ===");
            telemetry.addLine("DPAD: Edit | A: Add | B: Delete");
            telemetry.addLine("-----------------------------");
            telemetry.addData(selectedRow == 0 ? "-> ACTION" : "   ACTION", curType);
            telemetry.addData(selectedRow == 1 ? "-> POWER " : "   POWER ", "%.0f%%", curPower * 100);
            telemetry.addData(selectedRow == 2 ? "-> TIME  " : "   TIME  ", "%.1fs", curTime);
            telemetry.addData(selectedRow == 3 ? "-> DELAY " : "   DELAY ", "%.1fs (Shoot Only)", curDelay);
            telemetry.addLine("-----------------------------");
            telemetry.addLine("PROGRAM:");
            for (int i = 0; i < program.size(); i++) {
                telemetry.addData("" + (i + 1), program.get(i).toString());
            }
            telemetry.update();
        }

        // ==========================================
        // EXECUTION MODE (START PRESSED)
        // ==========================================
        if (opModeIsActive()) {
            for (AutoStep step : program) {
                if (!opModeIsActive()) break;
                telemetry.addData("Running", step.toString());
                telemetry.update();
                execute(step);
                sleep(200); // Brief pause to prevent skidding
            }
        }
    }

    private void execute(AutoStep step) {
        if (step.type == ActionType.SHOOT) {
            // SHOOTING LOGIC:
            // 1. Spin Shooter (Wait for Delay)
            shooter.setPower(step.power);
            sleep((long)(step.delay * 1000));

            // 2. Transfer + Intake FORWARD (1.0s)
            transfer.setPower(step.power);
            intake.setPower(step.power);
            sleep(1000);

            // 3. Intake REVERSE (0.3s) while Transfer keeps going
            intake.setPower(-step.power);
            sleep(300);

            // 4. Stop All
            shooter.setPower(0);
            transfer.setPower(0);
            intake.setPower(0);

        } else {
            // DRIVE LOGIC
            double y = 0, x = 0, rx = 0;
            double p = step.power;

            // Standard Mecanum Mapping
            // Forward = +y, Left = -x, Turn Right = +rx
            switch (step.type) {
                case FORWARD:   y = p;  break;
                case BACKWARD:  y = -p; break;
                case LEFT:      x = -p; break; // Strafe Left
                case RIGHT:     x = p;  break; // Strafe Right
                case TURN_LEFT: rx = -p; break;
                case TURN_RIGHT:rx = p; break;
            }

            // The Math (Standard)
            // If this moves wrong, DO NOT CHANGE MATH. Change .setDirection at top.
            double flP = y + x + rx;
            double blP = y - x + rx;
            double frP = y - x - rx;
            double brP = y + x - rx;

            // Normalize (ensure no motor gets > 1.0)
            double max = Math.max(Math.abs(flP), Math.max(Math.abs(blP), Math.max(Math.abs(frP), Math.abs(brP))));
            if (max > 1.0) {
                flP /= max; blP /= max; frP /= max; brP /= max;
            }

            fl.setPower(flP);
            bl.setPower(blP);
            fr.setPower(frP);
            br.setPower(brP);

            sleep((long) (step.seconds * 1000));

            // Stop
            fl.setPower(0); bl.setPower(0); fr.setPower(0); br.setPower(0);
        }
    }
}