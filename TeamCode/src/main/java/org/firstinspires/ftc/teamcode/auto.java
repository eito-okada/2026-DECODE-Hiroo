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

@Autonomous(name = "ネタが尽きた V12", group = "Main")
public class auto extends LinearOpMode {

    private DcMotor fl, fr, bl, br;
    private DcMotor gecko, intake;
    private IMU imu;

    private enum ActionType {
        FORWARD, BACKWARD, LEFT, RIGHT, TURN_LEFT, TURN_RIGHT,
        TRANSFER, INTAKE
    }

    private static class AutoStep {
        ActionType type;
        double power;
        double value;
        boolean joinNext;

        public AutoStep(ActionType t, double p, double v, boolean j) {
            this.type = t;
            this.power = p;
            this.value = v;
            this.joinNext = j;
        }

        @NonNull
        @Override
        public String toString() {
            String joinStr = joinNext ? " [+ JOIN]" : "";

            String dirStr = "";
            if (power < 0) dirStr = " (REV)";

            if (type == ActionType.TURN_LEFT || type == ActionType.TURN_RIGHT) {
                return String.format("%s (%.0f%%) %.0f deg%s", type, Math.abs(power * 100), value, joinStr);
            }
            return String.format("%s (%.0f%%%s) %.1fs%s", type, Math.abs(power * 100), dirStr, value, joinStr);
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
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        int selectedRow = 0;
        ActionType curType = ActionType.FORWARD;
        double curPower = 0.5;
        double curValue = 1.0;
        boolean curJoin = false;
        boolean curReverse = false; // Direction Toggle
        double globalShooterPower = 0.6;

        boolean lastUp = false, lastDown = false, lastLeft = false, lastRight = false;
        boolean lastA = false, lastB = false, lastX = false;

        ActionType[] types = ActionType.values();

        while (!isStarted() && !isStopRequested()) {

            // Rows: 0=Action, 1=Power, 2=Dir, 3=Time/Deg, 4=Join, 5=Flywheel
            if (gamepad1.dpad_up && !lastUp) selectedRow--;
            if (gamepad1.dpad_down && !lastDown) selectedRow++;
            if (selectedRow < 0) selectedRow = 5;
            if (selectedRow > 5) selectedRow = 0;

            if (gamepad1.dpad_right && !lastRight) {
                if (selectedRow == 0) {
                    curType = types[(curType.ordinal() + 1) % types.length];
                    if (curType == ActionType.TURN_LEFT || curType == ActionType.TURN_RIGHT) {
                        if (curValue < 10) curValue = 90;
                    } else {
                        if (curValue > 10) curValue = 1.0;
                    }
                }
                else if (selectedRow == 1) curPower = Math.min(1.0, curPower + 0.1);
                else if (selectedRow == 2) curReverse = !curReverse; // Toggle Dir
                else if (selectedRow == 3) {
                    if (curType == ActionType.TURN_LEFT || curType == ActionType.TURN_RIGHT) curValue += 5;
                    else curValue += 0.1;
                }
                else if (selectedRow == 4) curJoin = !curJoin;
                else globalShooterPower = Math.min(1.0, globalShooterPower + 0.1);
            }

            if (gamepad1.dpad_left && !lastLeft) {
                if (selectedRow == 0) {
                    int prev = curType.ordinal() - 1;
                    if (prev < 0) prev = types.length - 1;
                    curType = types[prev];
                }
                else if (selectedRow == 1) curPower = Math.max(0.1, curPower - 0.1);
                else if (selectedRow == 2) curReverse = !curReverse; // Toggle Dir
                else if (selectedRow == 3) {
                    if (curType == ActionType.TURN_LEFT || curType == ActionType.TURN_RIGHT) curValue = Math.max(5, curValue - 5);
                    else curValue = Math.max(0.1, curValue - 0.1);
                }
                else if (selectedRow == 4) curJoin = !curJoin;
                else globalShooterPower = Math.max(0.0, globalShooterPower - 0.1);
            }

            if (gamepad1.a && !lastA) {
                double finalPower = curReverse ? -curPower : curPower;
                program.add(new AutoStep(curType, finalPower, curValue, curJoin));
            }
            if (gamepad1.b && !lastB && !program.isEmpty()) program.remove(program.size() - 1);

            if (gamepad1.x && !lastX) {
                if (fl.getDirection() == DcMotorSimple.Direction.FORWARD) fl.setDirection(DcMotorSimple.Direction.REVERSE);
                else fl.setDirection(DcMotorSimple.Direction.FORWARD);
            }

            lastUp = gamepad1.dpad_up; lastDown = gamepad1.dpad_down;
            lastLeft = gamepad1.dpad_left; lastRight = gamepad1.dpad_right;
            lastA = gamepad1.a; lastB = gamepad1.b; lastX = gamepad1.x;

            telemetry.addLine("=== CREATOR V12 ===");
            telemetry.addLine("DPAD: Edit | A: Add | B: Delete");
            telemetry.addLine("X: Toggle FL Dir");
            telemetry.addLine();

            telemetry.addData(selectedRow == 0 ? "-> ACTION" : "   ACTION", curType);
            telemetry.addData(selectedRow == 1 ? "-> POWER " : "   POWER ", "%.0f%%", curPower * 100);
            telemetry.addData(selectedRow == 2 ? "-> DIR   " : "   DIR   ", curReverse ? "REVERSE" : "FORWARD");

            if (curType == ActionType.TURN_LEFT || curType == ActionType.TURN_RIGHT) {
                telemetry.addData(selectedRow == 3 ? "-> ANGLE " : "   ANGLE ", "%.0f deg", curValue);
            } else {
                telemetry.addData(selectedRow == 3 ? "-> TIME  " : "   TIME  ", "%.1fs", curValue);
            }

            telemetry.addData(selectedRow == 4 ? "-> JOIN  " : "   JOIN  ", curJoin ? "YES (Run w/ Next)" : "NO");

            telemetry.addLine();
            telemetry.addData(selectedRow == 5 ? "-> FLYWL " : "   FLYWL ", "%.0f%%", globalShooterPower * 100);

            telemetry.addLine("-----------------------------");
            telemetry.addData("FL Motor Dir", fl.getDirection());
            telemetry.addLine("-----------------------------");

            telemetry.addLine("PROGRAM:");
            for (int i = 0; i < program.size(); i++) {
                telemetry.addData("" + (i + 1), program.get(i).toString());
            }
            telemetry.update();
        }

        if (opModeIsActive()) {
            shooter.setPower(globalShooterPower);

            for (int i = 0; i < program.size(); i++) {
                if (!opModeIsActive()) break;

                AutoStep step = program.get(i);
                telemetry.addData("Running Step", i + 1);
                telemetry.addData("Action", step.toString());
                telemetry.update();

                startAction(step);

                if (step.joinNext) {
                    if (step.type == ActionType.TURN_LEFT || step.type == ActionType.TURN_RIGHT) {
                        waitForGyro(step.type == ActionType.TURN_LEFT ? step.value : -step.value, step.power);
                        stopAllMotors();
                    }
                } else {
                    if (step.type == ActionType.TURN_LEFT || step.type == ActionType.TURN_RIGHT) {
                        waitForGyro(step.type == ActionType.TURN_LEFT ? step.value : -step.value, step.power);
                    } else {
                        sleep((long) (step.value * 1000));
                    }
                    stopAllMotors();
                }
            }
            shooter.setPower(0);
            stopAllMotors();
        }
    }

    private void startAction(AutoStep step) {
        double p = step.power;

        switch (step.type) {
            case FORWARD:
                driveMecanum(p, 0, 0);
                break;
            case BACKWARD:
                driveMecanum(-p, 0, 0);
                break;
            case LEFT:
                driveMecanum(0, -p, 0);
                break;
            case RIGHT:
                driveMecanum(0, p, 0);
                break;

            case INTAKE:
                intake.setPower(p);
                break;

            case TRANSFER:
                // Runs BOTH Gecko and Intake
                gecko.setPower(p);
                intake.setPower(p);
                break;

            case TURN_LEFT:
            case TURN_RIGHT:
                break;
        }
    }

    private void stopAllMotors() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        gecko.setPower(0);
        intake.setPower(0);
    }

    private void waitForGyro(double targetAngleDeg, double power) {
        imu.resetYaw();
        while (opModeIsActive()) {
            double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double error = targetAngleDeg - currentAngle;
            if (Math.abs(error) < 2.0) break;
            double turnPower = Math.abs(power) * Math.signum(error);
            if (Math.abs(turnPower) < 0.15) turnPower = 0.15 * Math.signum(error);
            driveMecanum(0, 0, -turnPower);
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