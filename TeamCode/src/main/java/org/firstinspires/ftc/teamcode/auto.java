package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "誰が何を言おうと自動です V8", group = "Main")
public class auto extends LinearOpMode {

    private DcMotor fl, fr, bl, br;
    private DcMotor gecko, intake;
    private IMU imu;

    private enum ActionType { FORWARD, BACKWARD, LEFT, RIGHT, TURN_LEFT, TURN_RIGHT, TRANSFER }

    private static class AutoStep {
        ActionType type;
        double power;
        double value;

        public AutoStep(ActionType t, double p, double v) {
            this.type = t;
            this.power = p;
            this.value = v;
        }

        @NonNull
        @Override
        public String toString() {
            if (type == ActionType.TRANSFER) {
                return String.format("TRANSFER (Feed %.0f%%) for %.1fs", power * 100, value);
            } else if (type == ActionType.TURN_LEFT || type == ActionType.TURN_RIGHT) {
                return String.format("%s (%.0f%%) for %.0f deg", type, power * 100, value);
            }
            return String.format("%s (%.0f%%) for %.1fs", type, power * 100, value);
        }
    }

    private final List<AutoStep> program = new ArrayList<>();

    @Override
    public void runOpMode() {

        fl = hardwareMap.get(DcMotor.class, "motor0");
        bl = hardwareMap.get(DcMotor.class, "motor1");
        fr = hardwareMap.get(DcMotor.class, "motor2");
        br = hardwareMap.get(DcMotor.class, "motor3");

        intake = hardwareMap.get(DcMotor.class, "motor4");
        gecko = hardwareMap.get(DcMotor.class, "motor6");
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "motor7");

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        gecko.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        gecko.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        int selectedRow = 0;
        ActionType curType = ActionType.FORWARD;
        double curPower = 0.5;
        double curValue = 1.0;
        double globalShooterPower = 0.6;

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
                    curType = types[(curType.ordinal() + 1) % types.length];
                    if (curType == ActionType.TURN_LEFT || curType == ActionType.TURN_RIGHT) {
                        if (curValue < 10) curValue = 90;
                    } else {
                        if (curValue > 10) curValue = 1.0;
                    }
                } else if (selectedRow == 1) {
                    curPower = Math.min(1.0, curPower + 0.1);
                } else if (selectedRow == 2) {
                    if (curType == ActionType.TURN_LEFT || curType == ActionType.TURN_RIGHT) {
                        curValue += 5;
                    } else {
                        curValue += 0.1;
                    }
                } else {
                    globalShooterPower = Math.min(1.0, globalShooterPower + 0.1);
                }
            }

            if (gamepad1.dpad_left && !lastLeft) {
                if (selectedRow == 0) {
                    int prev = curType.ordinal() - 1;
                    if (prev < 0) prev = types.length - 1;
                    curType = types[prev];
                } else if (selectedRow == 1) {
                    curPower = Math.max(0.1, curPower - 0.1);
                } else if (selectedRow == 2) {
                    if (curType == ActionType.TURN_LEFT || curType == ActionType.TURN_RIGHT) {
                        curValue = Math.max(5, curValue - 5);
                    } else {
                        curValue = Math.max(0.1, curValue - 0.1);
                    }
                } else {
                    globalShooterPower = Math.max(0.0, globalShooterPower - 0.1);
                }
            }

            if (gamepad1.a && !lastA) program.add(new AutoStep(curType, curPower, curValue));
            if (gamepad1.b && !lastB && !program.isEmpty()) program.remove(program.size() - 1);

            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            lastLeft = gamepad1.dpad_left;
            lastRight = gamepad1.dpad_right;
            lastA = gamepad1.a;
            lastB = gamepad1.b;

            telemetry.addLine("=== CREATOR V8 ===");
            telemetry.addLine("DPAD: Edit | A: Add | B: Delete");
            telemetry.addLine();

            telemetry.addData(selectedRow == 0 ? "-> ACTION" : "   ACTION", curType);
            telemetry.addData(selectedRow == 1 ? "-> POWER " : "   POWER ", "%.0f%%", curPower * 100);

            if (curType == ActionType.TURN_LEFT || curType == ActionType.TURN_RIGHT) {
                telemetry.addData(selectedRow == 2 ? "-> ANGLE " : "   ANGLE ", "%.0f deg", curValue);
            } else {
                telemetry.addData(selectedRow == 2 ? "-> TIME  " : "   TIME  ", "%.1fs", curValue);
            }

            telemetry.addLine();
            telemetry.addData(selectedRow == 3 ? "-> FLYWL " : "   FLYWL ", "%.0f%%", globalShooterPower * 100);
            telemetry.addLine();

            telemetry.addLine("PROGRAM:");
            for (int i = 0; i < program.size(); i++) {
                telemetry.addData("" + (i + 1), program.get(i).toString());
            }
            telemetry.update();
        }

        if (opModeIsActive()) {
            shooter.setPower(globalShooterPower);

            for (AutoStep step : program) {
                if (!opModeIsActive()) break;
                telemetry.addData("Running", step.toString());
                telemetry.update();
                execute(step);
                sleep(200);
            }

            shooter.setPower(0);
        }
    }

    private void execute(AutoStep step) {
        if (step.type == ActionType.TRANSFER) {
            gecko.setPower(step.power);
            intake.setPower(step.power);
            sleep((long) (step.value * 1000));
            gecko.setPower(0);
            intake.setPower(0);
        } else if (step.type == ActionType.TURN_LEFT || step.type == ActionType.TURN_RIGHT) {
            turnToAngle(step.type == ActionType.TURN_LEFT ? step.value : -step.value, step.power);
        } else {
            double y = 0, x = 0;
            double p = step.power;

            switch (step.type) {
                case FORWARD: y = p; break;
                case BACKWARD: y = -p; break;
                case LEFT: x = -p; break;
                case RIGHT: x = p; break;
            }

            driveMecanum(y, x, 0);
            sleep((long) (step.value * 1000));
            driveMecanum(0, 0, 0);
        }
    }

    private void turnToAngle(double targetAngleDeg, double power) {
        imu.resetYaw();

        while (opModeIsActive()) {
            double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double error = targetAngleDeg - currentAngle;

            if (Math.abs(error) < 2.0) break;

            double turnPower = Math.abs(power) * Math.signum(error);
            if (Math.abs(turnPower) < 0.15) turnPower = 0.15 * Math.signum(error);

            driveMecanum(0, 0, -turnPower);

            telemetry.addData("Target", "%.1f", targetAngleDeg);
            telemetry.addData("Current", "%.1f", currentAngle);
            telemetry.update();
        }
        driveMecanum(0, 0, 0);
    }

    private void driveMecanum(double y, double x, double rx) {
        double flP = y + x + rx;
        double blP = y - x + rx;
        double frP = y - x - rx;
        double brP = y + x - rx;

        double max = Math.max(Math.abs(flP), Math.max(Math.abs(blP), Math.max(Math.abs(frP), Math.abs(brP))));
        if (max > 1.0) {
            flP /= max;
            blP /= max;
            frP /= max;
            brP /= max;
        }

        fl.setPower(flP);
        bl.setPower(blP);
        fr.setPower(frP);
        br.setPower(brP);
    }
}