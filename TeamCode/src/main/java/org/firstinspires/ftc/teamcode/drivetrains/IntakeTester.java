package org.firstinspires.ftc.teamcode.drivetrains;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "IntakeTester")
public class IntakeTester extends OpMode {
    private CRServo intake_left = null;
    private CRServo intake_right = null;
    @Override
    public void init()
    {
        intake_left = hardwareMap.get(CRServo.class, "Intake_Left");
        intake_right = hardwareMap.get(CRServo.class, "Intake_Right");
    }

    @Override
    public void loop()
    {
        intake_left.setPower(gamepad1.left_stick_y);
        intake_right.setPower(gamepad1.right_stick_y*-1);
    }
}
