package org.firstinspires.ftc.teamcode.opmode.roverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="HingeTesterWithRatios")
public class RuckusHingeTankArm extends RuckusTankBasic {
    @Override public void init()
    {
        for (RuckusMotorName m : winchMotors)
        {
            m.activate();
        }
        for (RuckusMotorName m : armMotor)
        {
            m.activate();
        }
        super.init();
    }

    @Override
    public void loop()
    {
        super.loop();
        setHingePower(gamepad1.left_trigger - gamepad1.right_trigger);
        setArmPower(gamepad2.left_trigger - gamepad2.right_trigger);

    }

}
