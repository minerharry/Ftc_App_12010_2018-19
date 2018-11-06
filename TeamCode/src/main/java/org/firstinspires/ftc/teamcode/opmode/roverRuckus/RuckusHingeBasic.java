package org.firstinspires.ftc.teamcode.opmode.roverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="HingeTesterWithRatios")
public class RuckusHingeBasic extends RuckusRobotHardware {
    @Override public void init()
    {
        for (RuckusMotorName m : winchMotors)
        {
            m.activate();
        }
        super.init();
    }

    @Override
    public void loop()
    {
        super.loop();
        //NOT OFFICIAL INPUTS, TESTING PURPOSES ONLY

        setHingePower(gamepad1.left_trigger-gamepad1.right_trigger);

    }

}
