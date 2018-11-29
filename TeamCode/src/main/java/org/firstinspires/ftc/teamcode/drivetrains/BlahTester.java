package org.firstinspires.ftc.teamcode.drivetrains;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="driveForward")
public class BlahTester extends OpMode {

    DcMotor motor = null;
    @Override
    public void init()
    {
        motor = hardwareMap.get(DcMotor.class,"Motor_Back_Left");
    }
    @Override
    public void loop()
    {
        motor.setPower(gamepad1.left_stick_y);
    }
}
