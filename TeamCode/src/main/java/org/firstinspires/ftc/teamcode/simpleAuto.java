package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


@Autonomous(name = "simple auto")
public class simpleAuto extends LinearOpMode {

    DcMotor fl, fr, bl, br;
    DcMotor transfer;
    DcMotorEx shooter;

    @Override
    public void runOpMode() {
        fl = hardwareMap.get(DcMotor.class, "motor0");
        bl = hardwareMap.get(DcMotor.class, "motor1");
        fr = hardwareMap.get(DcMotor.class, "motor2");
        br = hardwareMap.get(DcMotor.class, "motor3");
        fl.setDirection(DcMotorSimple.Direction.REVERSE); //ワンチャンなおす
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer = hardwareMap.get(DcMotor.class, "motor6");
        shooter = hardwareMap.get(DcMotorEx.class, "motor7");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(20, 0, 0, 3);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        waitForStart(); // The code stops here until you hit "PLAY"

        // Your movement logic goes here
        driveForward(-1, 2000); // Drive at 100% power backwards for 2 sec
    }
    public void driveForward(double power, int time) {
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(20, 0, 0, 3);
        shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        sleep(time);
        shooter.setVelocity(1520);
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
}