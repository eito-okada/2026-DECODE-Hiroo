package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Flywheel Tuner", group = "Main")
public class flywheeltuner extends OpMode {
    DcMotor transfer;
    public DcMotorEx flywheelmotor;

    public double highvelocity = 1520;
    public double lowvelocity = 900;

    double curTargetVelocity = highvelocity;

    double F = 0;
    double P = 0;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};

    int stepIndex = 1;


    @Override
    public void init() {
        transfer = hardwareMap.get(DcMotor.class, "motor6");
        flywheelmotor = hardwareMap.get(DcMotorEx.class, "motor7");
        flywheelmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelmotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Init Complete");
    }

    public void loop() {
        //commands
        if (gamepad1.yWasPressed()) {
            if (curTargetVelocity == highvelocity) {
                curTargetVelocity = lowvelocity;
            } else {
                curTargetVelocity = highvelocity;
            }
        }

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }

        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }

        if (gamepad1.x) { // Square
            transfer.setPower(-1.0);
        } else {
            transfer.setPower(0);
        }

        //set new PIDF coefficient
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelmotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        flywheelmotor.setVelocity(curTargetVelocity);

        double curVelocity = flywheelmotor.getVelocity();
        double error = curTargetVelocity - curVelocity;

        //telemetry
        telemetry.addData("Target Velocity:", curTargetVelocity);
        telemetry.addData("Current Velocity:", "%.2f", curVelocity);
        telemetry.addData("Error","%.2f", error);
        telemetry.addLine("-------------------------------------");
        telemetry.addData("Tuning P (D-pad U/D):", "%.4f", P);
        telemetry.addData("Tuning F (D-pad L/R):", "%.4f", F);
        telemetry.addData("Step Size (B Button):", "%.4f", stepSizes[stepIndex]);
    }
}
