package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "totally auto V2", group = "Main")
public class auto extends LinearOpMode {

    private DcMotor fl, fr, bl, br;
    private DcMotor shooter, transfer;

    private enum ActionType { FORWARD, BACKWARD, LEFT, RIGHT, SHOOT }

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
                return String.format("SHOOT %.0f%% (Wait %.1fs, Run %.1fs)", power * 100, delay, seconds);
            }
            return String.format("%s (%.0f%%) for %.1fs", type, power * 100, seconds);
        }
    }

    private List<AutoStep> program = new ArrayList<>();

    @Override
    public void runOpMode() {

        fl = hardwareMap.get(DcMotor.class, "motor0");
        bl = hardwareMap.get(DcMotor.class, "motor1");
        fr = hardwareMap.get(DcMotor.class, "motor2");
        br = hardwareMap.get(DcMotor.class, "motor3");
        shooter = hardwareMap.get(DcMotor.class, "motor7");
        transfer = hardwareMap.get(DcMotor.class, "motor6");

        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        int selectedRow = 0;
        ActionType curType = ActionType.FORWARD;
        double curPower = 0.5;
        double curTime = 1.0;
        double curDelay = 1.0;

        boolean lastUp = false, lastDown = false, lastLeft = false, lastRight = false;
        boolean lastA = false, lastB = false;

        ActionType[] types = ActionType.values();

        while (!isStarted() && !isStopRequested()) {

            if (gamepad1.dpad_up && !lastUp) selectedRow--;
            if (gamepad1.dpad_down && !lastDown) selectedRow++;
            if (selectedRow < 0) selectedRow = 3;
            if (selectedRow > 3) selectedRow = 0;

            if (gamepad1.dpad_right && !lastRight) {
                if (selectedRow == 0) {
                    int next = (curType.ordinal() + 1) % types.length;
                    curType = types[next];
                } else if (selectedRow == 1) {
                    curPower += 0.1;
                    if (curPower > 1.0) curPower = 1.0;
                } else if (selectedRow == 2) {
                    curTime += 0.1;
                } else {
                    curDelay += 0.1;
                }
            }

            if (gamepad1.dpad_left && !lastLeft) {
                if (selectedRow == 0) {
                    int prev = (curType.ordinal() - 1);
                    if (prev < 0) prev = types.length - 1;
                    curType = types[prev];
                } else if (selectedRow == 1) {
                    curPower -= 0.1;
                    if (curPower < 0.1) curPower = 0.1;
                } else if (selectedRow == 2) {
                    curTime -= 0.1;
                    if (curTime < 0.1) curTime = 0.1;
                } else {
                    curDelay -= 0.1;
                    if (curDelay < 0.0) curDelay = 0.0;
                }
            }

            if (gamepad1.a && !lastA) {
                program.add(new AutoStep(curType, curPower, curTime, curDelay));
            }

            if (gamepad1.b && !lastB) {
                if (!program.isEmpty()) {
                    program.remove(program.size() - 1);
                }
            }

            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            lastLeft = gamepad1.dpad_left;
            lastRight = gamepad1.dpad_right;
            lastA = gamepad1.a;
            lastB = gamepad1.b;

            telemetry.addLine("=== CREATOR MODE ===");
            telemetry.addLine("DPAD: Change | A: Add | B: Delete");
            telemetry.addLine();

            telemetry.addData(selectedRow == 0 ? "-> ACTION" : "   ACTION", curType);
            telemetry.addData(selectedRow == 1 ? "-> POWER " : "   POWER ", "%.0f%%", curPower * 100);
            telemetry.addData(selectedRow == 2 ? "-> TIME  " : "   TIME  ", "%.1fs", curTime);
            telemetry.addData(selectedRow == 3 ? "-> DELAY " : "   DELAY ", "%.1fs (Shoot Only)", curDelay);

            telemetry.addLine();
            telemetry.addLine("--- CURRENT PROGRAM ---");
            for (int i = 0; i < program.size(); i++) {
                telemetry.addData(String.valueOf(i + 1), program.get(i).toString());
            }
            telemetry.update();
        }

        if (opModeIsActive()) {
            for (AutoStep step : program) {
                if (!opModeIsActive()) break;
                telemetry.addData("Executing", step.toString());
                telemetry.update();
                execute(step);
                sleep(200);
            }
        }
    }

    private void execute(AutoStep step) {
        if (step.type == ActionType.SHOOT) {
            shooter.setPower(step.power);
            sleep((long)(step.delay * 1000));

            transfer.setPower(step.power);
            sleep((long)(step.seconds * 1000));

            shooter.setPower(0);
            transfer.setPower(0);
        } else {
            double y = 0, x = 0, rx = 0;
            double p = step.power;

            switch (step.type) {
                case FORWARD: y = p; break;
                case BACKWARD: y = -p; break;
                case LEFT: x = -p; break;
                case RIGHT: x = p; break;
            }

            double flP = y + x + rx;
            double frP = y - x - rx;
            double blP = y - x + rx;
            double brP = y + x - rx;

            fl.setPower(flP);
            fr.setPower(frP);
            bl.setPower(blP);
            br.setPower(brP);

            sleep((long) (step.seconds * 1000));

            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);
        }
    }
}