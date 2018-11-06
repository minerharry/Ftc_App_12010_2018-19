package org.firstinspires.ftc.teamcode.opmode.roverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

public class RuckusTankArmIntakeHinge extends RuckusTankBasic {
    @Override
    public void init()
    {
        super.init();
        for(RuckusMotorName m : winchMotors)
        {
            m.activate();
        }
        for(RuckusMotorName m : armMotor)
        {
            m.activate();
        }
        for(RuckusCRServoName s : intakeServos)
        {
            s.activate();
        }
    }

    @Override
    public void loop()
    {
        super.loop();
        if (gamepad1.right_bumper)
            setIntakeState(false);
        else
            if (gamepad1.left_bumper)
                setIntakeState(true);
        setArmPower(gamepad1.left_trigger - gamepad1.right_trigger);
    }
}
