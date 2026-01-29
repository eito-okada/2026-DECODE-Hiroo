package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class flywheeltuner extends OpMode {

    public DcMotorEx flywheelmotor;

    public double highvelocity = 4500;
    public double lowvelocity = 2000;

    double curTargetVelocity = highvelocity;

    double F = 0;
    double P = 0;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};

    int stepIndex = 1;


    @Override
    public void init() {
        flywheelmotor = hardwareMap.get(DcMotorEx.class, "motor");
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

        //set new PIDF coefficient
        flywheelmotor.setVelocity(curTargetVelocity);

        double curVelocity = flywheelmotor.getVelocity();
        double error = curTargetVelocity - curVelocity;

        //telemetry
        telemetry.addData("Target Velocity:", curTargetVelocity);
        telemetry.addData("Current Velocity:", "%.2f", curVelocity);
        telemetry.addData("Error","%.2f", error);
        telemetry.addLine("-------------------------------------");
        telemetry.addData("Tuning P:", "&.4f (D-pad U/D)", P);
        telemetry.addData("Tuning F:", "&.4f (D-pad L/R)", F);
        telemetry.addData("Step Size:", "%.4f (B Button)", stepSizes[stepIndex]);
    }
}
