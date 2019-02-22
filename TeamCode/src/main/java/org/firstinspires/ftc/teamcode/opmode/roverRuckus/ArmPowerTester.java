package org.firstinspires.ftc.teamcode.opmode.roverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Arm Power Tester")
public class ArmPowerTester extends RuckusTankBasic {
    public void init()
    {
        armMotor[0].activate();
        super.init();
    }
    public void loop()
    {
        super.loop();
        setPower(armMotor[0],gamepad2.left_stick_y);
    }
}
