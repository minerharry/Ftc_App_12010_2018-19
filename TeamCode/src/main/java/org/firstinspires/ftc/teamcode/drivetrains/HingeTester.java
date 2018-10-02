package org.firstinspires.ftc.teamcode.drivetrains;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Hinge Arm OpMode Teleop")
public class HingeTester extends OpMode {

    private DcMotor hingeMotor = null;
    private String hingeMotorName = "hingeMotor";

    @Override
    public void init()
    {
        hingeMotor = hardwareMap.get(DcMotor.class, hingeMotorName);
        telemetry.addData("Initialized","Yeah");
    }
    @Override
    public void loop()
    {
        hingeMotor.setPower(gamepad1.left_stick_y*0.5);
    }
}
