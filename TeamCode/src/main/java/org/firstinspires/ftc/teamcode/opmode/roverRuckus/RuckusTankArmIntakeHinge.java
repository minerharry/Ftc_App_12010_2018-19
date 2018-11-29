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
        for (RuckusServoName m : armSlideServo)
        {
            m.activate();
        }
        setIntakeType(true);
    }

    @Override
    public void loop()
    {
        super.loop();
        setHingePower(gamepad1.left_trigger - gamepad1.right_trigger);
        setArmPower(gamepad2.left_trigger - gamepad2.right_trigger);
        setIntakePower(gamepad1.left_stick_y);
        incrementScoop(gamepad2.right_stick_y);
        slideArm((gamepad2.left_bumper ? 1 : 0) + (gamepad2.right_bumper ? -1 : 0));
    }
}
