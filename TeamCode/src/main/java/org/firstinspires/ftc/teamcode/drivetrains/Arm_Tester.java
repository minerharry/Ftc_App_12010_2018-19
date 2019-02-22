package org.firstinspires.ftc.teamcode.drivetrains;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Hardware;

@TeleOp(name="armNoEncoderTest")
public class Arm_Tester extends OpMode {
    public DcMotor armMotor = null;
    public void init()
    {
        armMotor = hardwareMap.get(DcMotor.class,"Motor_Arm");
    }
    public void loop()
    {
        armMotor.setPower(gamepad1.left_stick_y);
    }
}
