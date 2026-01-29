package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "MotorTest")
public class Test extends OpMode {
    private DcMotorEx motor0;
    @Override
    public void init() {
        motor0 = hardwareMap.get(DcMotorEx.class,"motor0");
    }

    @Override
    public void loop() {
        motor0.setPower(0.5);
    }
}
