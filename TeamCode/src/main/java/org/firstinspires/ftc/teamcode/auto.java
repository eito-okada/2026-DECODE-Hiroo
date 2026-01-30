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

@Autonomous(name = "かわいい V16", group = "Main")
public class auto extends LinearOpMode {

    private DcMotor fl, fr, bl, br;
    private DcMotor gecko, intake;
    public DcMotorEx shooter;
    private IMU imu;

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

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        gecko.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

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
        boolean curReverse = false;

        double globalShooterPower = 0.6;
        boolean globalShooterReverse = false;

        boolean lastUp = false, lastDown = false, lastLeft = false, lastRight = false;
        boolean lastA = false, lastB = false;

        ActionType[] types = ActionType.values();

        while (!isStarted() && !isStopRequested()) {

            // Rows: 0=Action, 1=Power, 2=Dir, 3=Time/Deg, 4=Join, 5=Flywl Speed, 6=Flywl Dir, 7=PRESET
            if (gamepad1.dpad_up && !lastUp) selectedRow--;
            if (gamepad1.dpad_down && !lastDown) selectedRow++;
            if (selectedRow < 0) selectedRow = 7;
            if (selectedRow > 7) selectedRow = 0;

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
                else if (selectedRow == 2) curReverse = !curReverse;
                else if (selectedRow == 3) {
                    if (curType == ActionType.TURN_LEFT || curType == ActionType.TURN_RIGHT) curValue += 5;
                    else curValue += 0.1;
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
                }
                else if (selectedRow == 1) curPower = Math.max(0.1, curPower - 0.1);
                else if (selectedRow == 2) curReverse = !curReverse;
                else if (selectedRow == 3) {
                    if (curType == ActionType.TURN_LEFT || curType == ActionType.TURN_RIGHT) curValue = Math.max(5, curValue - 5);
                    else curValue = Math.max(0.1, curValue - 0.1);
                }
                else if (selectedRow == 4) curJoin = !curJoin;
                else if (selectedRow == 5) globalShooterPower = Math.max(0.0, globalShooterPower - 0.1);
                else if (selectedRow == 6) globalShooterReverse = !globalShooterReverse;
            }

            // ADD BUTTON LOGIC (Handles Preset Loading)
            if (gamepad1.a && !lastA) {
                if (selectedRow == 7) {
                    // --- LOAD PRESET ---
                    // 1. Backward 30% 3.5s
                    program.add(new AutoStep(ActionType.BACKWARD, 0.3, 3.5, false));
                    // 2. Gecko 100% 2s Forward
                    program.add(new AutoStep(ActionType.GECKO, 1.0, 2.0, false));
                    // 3. Intake 100% 0.3s Reverse (-1.0 Power)
                    program.add(new AutoStep(ActionType.INTAKE, -1.0, 0.3, false));
                    // 4. Intake 100% 5s Forward JOIN=ON
                    program.add(new AutoStep(ActionType.INTAKE, 1.0, 5.0, true));
                    // 5. Gecko 100% 5s Forward
                    program.add(new AutoStep(ActionType.GECKO, 1.0, 5.0, false));
                } else {
                    // --- STANDARD ADD ---
                    double finalPower = curReverse ? -curPower : curPower;
                    program.add(new AutoStep(curType, finalPower, curValue, curJoin));
                }
            }

            if (gamepad1.b && !lastB && !program.isEmpty()) program.remove(program.size() - 1);

            lastUp = gamepad1.dpad_up; lastDown = gamepad1.dpad_down;
            lastLeft = gamepad1.dpad_left; lastRight = gamepad1.dpad_right;
            lastA = gamepad1.a; lastB = gamepad1.b;

            telemetry.addLine("=== CREATOR V16 (PRESET) ===");
            telemetry.addLine("DPAD: Edit | A: Add | B: Delete");
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
            telemetry.addData(selectedRow == 5 ? "-> FLYWL SPD" : "   FLYWL SPD", "%.0f%%", globalShooterPower * 100);
            telemetry.addData(selectedRow == 6 ? "-> FLYWL DIR" : "   FLYWL DIR", globalShooterReverse ? "REVERSE" : "FORWARD");

            telemetry.addLine("-----------------------------");
            if (selectedRow == 7) {
                telemetry.addLine("-> [LOAD PRESET 1] <-");
                telemetry.addLine("   (Back, Gecko, Intake Seq)");
            } else {
                telemetry.addLine("   [LOAD PRESET 1]");
            }
            telemetry.addLine("-----------------------------");

            telemetry.addLine("PROGRAM:");
            for (int i = 0; i < program.size(); i++) {
                telemetry.addData("" + (i + 1), program.get(i).toString());
            }
            telemetry.update();
        }

        if (opModeIsActive()) {

            if (globalShooterReverse) {
                shooter.setDirection(DcMotorSimple.Direction.REVERSE);
            } else {
                shooter.setDirection(DcMotorSimple.Direction.FORWARD);
            }
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
            case FORWARD:   driveMecanum(p, 0, 0); break;
            case BACKWARD:  driveMecanum(-p, 0, 0); break;
            case LEFT:      driveMecanum(0, -p, 0); break;
            case RIGHT:     driveMecanum(0, p, 0); break;

            case INTAKE:
                intake.setPower(p);
                break;
            case GECKO:
                gecko.setPower(p);
                break;
            case TRANSFER:
                gecko.setPower(p);
                intake.setPower(p);
                break;

            case TURN_LEFT:
            case TURN_RIGHT:
                break;
        }
    }

    private void stopAllMotors() {
        fl.setPower(0); fr.setPower(0); bl.setPower(0); br.setPower(0);
        gecko.setPower(0); intake.setPower(0);
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
        if (max > 1.0) { flP /= max; blP /= max; frP /= max; brP /= max; }
        fl.setPower(flP); bl.setPower(blP); fr.setPower(frP); br.setPower(brP);
    }
}