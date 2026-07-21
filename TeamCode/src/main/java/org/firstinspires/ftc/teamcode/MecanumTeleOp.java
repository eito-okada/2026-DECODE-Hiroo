package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Shooter;

/**
 * Final-version driver-controlled mecanum drive plus a closed-loop velocity shooter.
 *
 * Hardware map:
 *   motor0 = intake
 *   motor1 = LF
 *   motor2 = LB
 *   motor3 = (empty)
 *   motor4 = RF
 *   motor5 = RB
 *   motor6 = shooter 1  (carries the encoder, closed velocity loop)
 *   motor7 = shooter 2   (same direction as shooter 1 — shared chain, mirrors shooter 1's power)
 *   servo0 = transfer stopper (90 deg rest / 0 deg turned)
 *
 * Controls (gamepad1):
 *   left stick / right stick x .. drive / turn
 *   right trigger ............... intake (hold, full power)
 *   right bumper ................ toggle both shooters on/off at target RPM
 *   a ............................ toggle the transfer gate between 90 deg and 0 deg
 */
@TeleOp(name = "Mecanum Drive", group = "Main")
public class MecanumTeleOp extends OpMode {

    private static final String INTAKE_MOTOR_NAME = "motor0";
    private static final double INTAKE_POWER = 1.0;

    private static final String SHOOTER_A_NAME = "motor6";
    private static final String SHOOTER_B_NAME = "motor7";
    private static final double SHOOTER_TARGET_RPM = 3200.0;
    private static final double SHOOTER_P = 25.0;
    private static final double SHOOTER_I = 100.0;
    private static final double SHOOTER_D = 0.0;
    private static final double SHOOTER_F = 14.0;

    // 300-degree servo: position = angle / 300.
    private static final String GATE_SERVO_NAME = "servo0";
    private static final double GATE_REST = 90.0 / 300.0;
    private static final double GATE_TURNED = 0.0 / 300.0;

    private DcMotor lf, lb, rf, rb;
    private DcMotor intake;
    private DcMotorEx shooterA, shooterB;
    private Servo gate;
    private boolean gateTurned = false;
    private boolean prevA;
    private boolean shooterOn = false;
    private boolean prevRightBumper;

    private static final double DEADZONE = 0.05;
    private static final double TRIGGER_THRESHOLD = 0.3;

    @Override
    public void init() {
        lf = hardwareMap.get(DcMotor.class, "motor1");
        lb = hardwareMap.get(DcMotor.class, "motor2");
        rf = hardwareMap.get(DcMotor.class, "motor4");
        rb = hardwareMap.get(DcMotor.class, "motor5");

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setDirection(DcMotorSimple.Direction.FORWARD);
        rb.setDirection(DcMotorSimple.Direction.FORWARD);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.get(DcMotor.class, INTAKE_MOTOR_NAME);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterA = hardwareMap.get(DcMotorEx.class, SHOOTER_A_NAME);
        shooterB = hardwareMap.get(DcMotorEx.class, SHOOTER_B_NAME);
        shooterA.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterB.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // shooterA carries the encoder and runs the closed velocity loop; shooterB has no
        // encoder and just mirrors shooterA's applied power (same pattern as Shooter.java).
        shooterA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterA.setVelocityPIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
        shooterB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        gate = hardwareMap.get(Servo.class, GATE_SERVO_NAME);
        gate.setPosition(GATE_REST);

        telemetry.addLine("Ready. RT = intake, RB = fire (shooters), A = toggle gate. Press START.");
    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double r = gamepad1.right_stick_x;

        if (Math.abs(y) < DEADZONE) y = 0;
        if (Math.abs(x) < DEADZONE) x = 0;
        if (Math.abs(r) < DEADZONE) r = 0;

        double lfPow = y + x + r;
        double rfPow = y - x - r;
        double lbPow = y - x + r;
        double rbPow = y + x - r;

        double max = Math.max(1.0, Math.max(Math.abs(lfPow),
                Math.max(Math.abs(rfPow), Math.max(Math.abs(lbPow), Math.abs(rbPow)))));

        lf.setPower(lfPow / max);
        rf.setPower(rfPow / max);
        lb.setPower(lbPow / max);
        rb.setPower(rbPow / max);

        // Intake on the right trigger.
        intake.setPower(gamepad1.right_trigger > TRIGGER_THRESHOLD ? INTAKE_POWER : 0);

        // Shooters: each right bumper press toggles both on/off.
        if (gamepad1.right_bumper && !prevRightBumper) {
            shooterOn = !shooterOn;
        }
        prevRightBumper = gamepad1.right_bumper;
        shooterA.setVelocity(shooterOn ? Shooter.rpmToTicksPerSec(SHOOTER_TARGET_RPM) : 0);
        shooterB.setPower(shooterA.getPower());

        // Gate: each A press toggles between 90 deg (rest) and 0 deg (turned).
        if (gamepad1.a && !prevA) {
            gateTurned = !gateTurned;
            gate.setPosition(gateTurned ? GATE_TURNED : GATE_REST);
        }
        prevA = gamepad1.a;

        telemetry.addData("intake (RT)", "%.2f", gamepad1.right_trigger);
        telemetry.addData("shooters", shooterOn
                ? String.format("ON  target=%.0f RPM  meas=%.0f RPM", SHOOTER_TARGET_RPM,
                        Shooter.ticksPerSecToRpm(shooterA.getVelocity()))
                : "off");
        telemetry.addData("gate", gateTurned ? "TURNED (0 deg)" : "rest (90 deg)");
        telemetry.addData("LF", "%.2f", lfPow / max);
        telemetry.addData("RF", "%.2f", rfPow / max);
        telemetry.addData("LB", "%.2f", lbPow / max);
        telemetry.addData("RB", "%.2f", rbPow / max);
        telemetry.update();
    }

    @Override
    public void stop() {
        intake.setPower(0);
        shooterA.setPower(0);
        shooterB.setPower(0);
    }
}
