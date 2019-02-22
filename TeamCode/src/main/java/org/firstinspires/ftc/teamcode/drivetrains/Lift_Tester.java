package org.firstinspires.ftc.teamcode.drivetrains;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Hardware;

@TeleOp(name="Left Tester")
public class Lift_Tester extends OpMode {
    public DcMotor liftMotor;
    public void init()
    {
        liftMotor = hardwareMap.get(DcMotor.class,"Motor_Linear_Slide");
    }
    public void loop()
    {
        liftMotor.setPower(gamepad1.left_stick_y);
    }
}
