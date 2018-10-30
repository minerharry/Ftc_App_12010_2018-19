package org.firstinspires.ftc.teamcode.drivetrains;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "IntakeTester")
public class IntakeTester extends OpMode {
    private CRServo leftIntake = null;
    private CRServo rightIntake = null;

    @Override
    public void init()
    {
        leftIntake = hardwareMap.get(CRServo.class, "Left_Intake");
        rightIntake = hardwareMap.get(CRServo.class, "Right_Intake");
    }

    @Override
    public void loop()
    {
        double power = gamepad1.left_stick_y;
        leftIntake.setPower(power);
        telemetry.addData("Power", power);
    }

}
