package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "本当に究極の V21 (CM Version)", group = "Main")
public class auto extends LinearOpMode {

    private DcMotorEx fl, fr, bl, br;
    private DcMotor gecko, intake;
    public DcMotorEx shooter;
    private IMU imu;

    final double TICKS_PER_CM = 18.0;
    final double STRAFE_MULTI = 1.1;

    final double P_GAIN = 0.03;
    final double MIN_POWER = 0.13;
    final double HEADING_THRESHOLD = 0.5;

    private final double[] validAngles = {45.0, 90.0, 135.0, 180.0};
    private int angleIdx = 1;

    private enum ActionType {
        FORWARD, BACKWARD, LEFT, RIGHT, TURN_LEFT, TURN_RIGHT,
        INTAKE, GECKO, TRANSFER
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

            if (type == ActionType.TURN_LEFT || type == ActionType.TURN_RIGHT) {
                return String.format("%s (%.0f%%) %.0f deg%s", type, Math.abs(power * 100), value, joinStr);
            }
            else if (type == ActionType.FORWARD || type == ActionType.BACKWARD ||
                    type == ActionType.LEFT || type == ActionType.RIGHT) {
                return String.format("%s (%.0f%%) %.1f cm%s", type, Math.abs(power * 100), value, joinStr);
            }
            else {
                return String.format("%s (%.0f%%) %.1fs%s", type, Math.abs(power * 100), value, joinStr);
            }
        }
    }

    private final List<AutoStep> program = new ArrayList<>();

    @Override
    public void runOpMode() {

        fl = hardwareMap.get(DcMotorEx.class, "motor0");
        bl = hardwareMap.get(DcMotorEx.class, "motor1");
        fr = hardwareMap.get(DcMotorEx.class, "motor2");
        br = hardwareMap.get(DcMotorEx.class, "motor3");

        intake = hardwareMap.get(DcMotor.class, "motor4");
        gecko = hardwareMap.get(DcMotor.class, "motor6");
        shooter = hardwareMap.get(DcMotorEx.class, "motor7");

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(300, 0, 0, 30);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        gecko.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetDriveEncoders();

        int selectedRow = 0;
        ActionType curType = ActionType.FORWARD;
        double curPower = 0.5;
        double curValue = 30.0;
        boolean curJoin = false;
        boolean curReverse = false;

        double globalShooterPower = 0.6;
        boolean globalShooterReverse = false;

        boolean lastUp = false, lastDown = false, lastLeft = false, lastRight = false;
        boolean lastA = false, lastB = false;

        ActionType[] types = ActionType.values();

        while (!isStarted() && !isStopRequested()) {

            if (gamepad1.dpad_up && !lastUp) selectedRow--;
            if (gamepad1.dpad_down && !lastDown) selectedRow++;
            if (selectedRow < 0) selectedRow = 7;
            if (selectedRow > 7) selectedRow = 0;

            if (gamepad1.dpad_right && !lastRight) {
                if (selectedRow == 0) {
                    curType = types[(curType.ordinal() + 1) % types.length];
                    if (isTurn(curType)) curValue = validAngles[angleIdx];
                    else if (isDrive(curType)) curValue = 30.0;
                    else curValue = 1.0;
                }
                else if (selectedRow == 1) curPower = Math.min(1.0, curPower + 0.1);
                else if (selectedRow == 2) curReverse = !curReverse;
                else if (selectedRow == 3) {
                    if (isTurn(curType)) {
                        angleIdx = (angleIdx + 1) % validAngles.length;
                        curValue = validAngles[angleIdx];
                    } else if (isDrive(curType)) {
                        curValue += 1.0;
                    } else {
                        curValue += 0.1;
                    }
                }
                else if (selectedRow == 4) curJoin = !curJoin;
                else if (selectedRow == 5) globalShooterPower = Math.min(1.0, globalShooterPower + 0.1);
                else if (selectedRow == 6) globalShooterReverse = !globalShooterReverse;
            }

            if (gamepad1.dpad_left && !lastLeft) {
                if (selectedRow == 0) {
                    int prev = curType.ordinal() - 1;
                    if (prev < 0) prev = types.length - 1;
                    curType = types[prev];

                    if (isTurn(curType)) curValue = validAngles[angleIdx];
                    else if (isDrive(curType)) curValue = 30.0;
                    else curValue = 1.0;
                }
                else if (selectedRow == 1) curPower = Math.max(0.1, curPower - 0.1);
                else if (selectedRow == 2) curReverse = !curReverse;
                else if (selectedRow == 3) {
                    if (isTurn(curType)) {
                        angleIdx--;
                        if (angleIdx < 0) angleIdx = validAngles.length - 1;
                        curValue = validAngles[angleIdx];
                    } else if (isDrive(curType)) {
                        curValue = Math.max(1.0, curValue - 1.0);
                    } else {
                        curValue = Math.max(0.1, curValue - 0.1);
                    }
                }
                else if (selectedRow == 4) curJoin = !curJoin;
                else if (selectedRow == 5) globalShooterPower = Math.max(0.0, globalShooterPower - 0.1);
                else if (selectedRow == 6) globalShooterReverse = !globalShooterReverse;
            }

            if (gamepad1.a && !lastA) {
                if (selectedRow == 7) {
                    program.add(new AutoStep(ActionType.BACKWARD, 0.3, 20.0, false));
                    program.add(new AutoStep(ActionType.GECKO, 1.0, 2.0, false));
                    program.add(new AutoStep(ActionType.INTAKE, 1.0, 1.0, false));
                    program.add(new AutoStep(ActionType.INTAKE, -1.0, 0.3, false));
                    program.add(new AutoStep(ActionType.INTAKE, 1.0, 5.0, true));
                    program.add(new AutoStep(ActionType.GECKO, 1.0, 5.0, false));
                } else {
                    double finalPower = curReverse ? -curPower : curPower;
                    program.add(new AutoStep(curType, finalPower, curValue, curJoin));
                }
            }

            if (gamepad1.b && !lastB && !program.isEmpty()) program.remove(program.size() - 1);

            lastUp = gamepad1.dpad_up; lastDown = gamepad1.dpad_down;
            lastLeft = gamepad1.dpad_left; lastRight = gamepad1.dpad_right;
            lastA = gamepad1.a; lastB = gamepad1.b;

            telemetry.addLine("=== CREATOR V21 (CM VERSION) ===");
            telemetry.addLine("DPAD: Edit | A: Add | B: Delete");
            telemetry.addLine();

            telemetry.addData(selectedRow == 0 ? "-> ACTION" : "   ACTION", curType);
            telemetry.addData(selectedRow == 1 ? "-> POWER " : "   POWER ", "%.0f%%", curPower * 100);
            telemetry.addData(selectedRow == 2 ? "-> DIR   " : "   DIR   ", curReverse ? "REVERSE" : "FORWARD");

            if (isTurn(curType)) {
                telemetry.addData(selectedRow == 3 ? "-> ANGLE " : "   ANGLE ", "%.0f deg", curValue);
            } else if (isDrive(curType)) {
                telemetry.addData(selectedRow == 3 ? "-> DIST  " : "   DIST  ", "%.1f cm", curValue);
            } else {
                telemetry.addData(selectedRow == 3 ? "-> TIME  " : "   TIME  ", "%.1fs", curValue);
            }

            telemetry.addData(selectedRow == 4 ? "-> JOIN  " : "   JOIN  ", curJoin ? "YES (Run w/ Next)" : "NO");

            telemetry.addLine();
            telemetry.addData(selectedRow == 5 ? "-> FLYWL SPD" : "   FLYWL SPD", "%.0f%%", globalShooterPower * 100);
            telemetry.addData(selectedRow == 6 ? "-> FLYWL DIR" : "   FLYWL DIR", globalShooterReverse ? "REVERSE" : "FORWARD");

            telemetry.addLine("-----------------------------");
            if (selectedRow == 7) telemetry.addLine("-> [LOAD PRESET] <-");
            else telemetry.addLine("   [LOAD PRESET]");
            telemetry.addLine("-----------------------------");

            telemetry.addLine("PROGRAM:");
            for (int i = 0; i < program.size(); i++) {
                telemetry.addData("" + (i + 1), program.get(i).toString());
            }
            telemetry.update();
        }

        if (opModeIsActive()) {
            shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            shooter.setVelocity(1520);

            for (int i = 0; i < program.size(); i++) {
                if (!opModeIsActive()) break;

                AutoStep step = program.get(i);
                telemetry.addData("Running Step", i + 1);
                telemetry.addData("Action", step.toString());
                telemetry.update();

                if (isDrive(step.type)) {
                    driveEncoder(step.type, step.value, step.power);
                }
                else if (isTurn(step.type)) {
                    double target = (step.type == ActionType.TURN_LEFT) ? step.value : -step.value;
                    waitForGyro(target, step.power);
                }
                else {
                    startNonDriveAction(step);
                    if (!step.joinNext) {
                        sleep((long) (step.value * 1000));
                        stopAllMotors();
                    }
                }
            }
            shooter.setPower(0);
            stopAllMotors();
        }
    }

    private void driveEncoder(ActionType type, double cm, double power) {
        resetDriveEncoders();

        double targetTicks = cm * TICKS_PER_CM;
        if (type == ActionType.LEFT || type == ActionType.RIGHT) {
            targetTicks *= STRAFE_MULTI;
        }

        int flT = 0, frT = 0, blT = 0, brT = 0;
        int t = (int) targetTicks;

        switch (type) {
            case FORWARD:  flT = t;  frT = t;  blT = t;  brT = t;  break;
            case BACKWARD: flT = -t; frT = -t; blT = -t; brT = -t; break;
            case LEFT:     flT = -t; frT = t;  blT = t;  brT = -t; break;
            case RIGHT:    flT = t;  frT = -t; blT = -t; brT = t;  break;
        }

        setTargetPos(flT, frT, blT, brT);

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        driveMecanumRaw(power);

        while (opModeIsActive() && (fl.isBusy() && fr.isBusy() && bl.isBusy() && br.isBusy())) {
            telemetry.addData("Target", t);
            telemetry.addData("Current", fl.getCurrentPosition());
            telemetry.update();
        }

        stopAllMotors();
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void startNonDriveAction(AutoStep step) {
        double p = step.power;
        switch (step.type) {
            case INTAKE:   intake.setPower(p); break;
            case GECKO:    gecko.setPower(p); break;
            case TRANSFER: gecko.setPower(p); intake.setPower(p); break;
        }
    }

    private void waitForGyro(double targetAngleDeg, double maxPower) {
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu.resetYaw();
        long startTime = System.currentTimeMillis();
        long timeoutMs = 5000;

        while (opModeIsActive() && (System.currentTimeMillis() - startTime < timeoutMs)) {
            double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double error = targetAngleDeg - currentAngle;

            if (Math.abs(error) <= HEADING_THRESHOLD) break;

            double turnSpeed = error * P_GAIN;
            turnSpeed = Math.max(-maxPower, Math.min(maxPower, turnSpeed));

            if (Math.abs(turnSpeed) < MIN_POWER) turnSpeed = Math.signum(turnSpeed) * MIN_POWER;

            driveMecanumRawTurn(-turnSpeed);
        }
        stopAllMotors();
        sleep(150);
    }

    private void resetDriveEncoders() {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void setTargetPos(int flt, int frt, int blt, int brt) {
        fl.setTargetPosition(flt);
        fr.setTargetPosition(frt);
        bl.setTargetPosition(blt);
        br.setTargetPosition(brt);
    }

    private void driveMecanumRaw(double power) {
        fl.setPower(power); fr.setPower(power); bl.setPower(power); br.setPower(power);
    }

    private void driveMecanumRawTurn(double rx) {
        fl.setPower(rx); bl.setPower(rx); fr.setPower(-rx); br.setPower(-rx);
    }

    private void stopAllMotors() {
        fl.setPower(0); fr.setPower(0); bl.setPower(0); br.setPower(0);
        gecko.setPower(0); intake.setPower(0);
    }

    private boolean isDrive(ActionType t) {
        return t == ActionType.FORWARD || t == ActionType.BACKWARD || t == ActionType.LEFT || t == ActionType.RIGHT;
    }

    private boolean isTurn(ActionType t) {
        return t == ActionType.TURN_LEFT || t == ActionType.TURN_RIGHT;
    }
}