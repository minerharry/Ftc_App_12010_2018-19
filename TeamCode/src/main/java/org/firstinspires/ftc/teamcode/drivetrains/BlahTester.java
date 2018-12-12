package org.firstinspires.ftc.teamcode.drivetrains;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="ServoTest")
public class BlahTester extends OpMode {

    Servo servo = null;
    int frame = 0;
    @Override
    public void init()
    {
        servo = hardwareMap.get(Servo.class,"Servo_Arm_Slide");
    }
    @Override
    public void loop()
    {
        frame = (frame == 2 ? 0 : frame+1);
        if (frame==0){servo.setPosition(gamepad1.left_stick_y/2+0.5);}

    }
}
