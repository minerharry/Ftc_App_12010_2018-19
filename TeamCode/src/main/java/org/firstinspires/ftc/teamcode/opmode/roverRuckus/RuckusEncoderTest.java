package org.firstinspires.ftc.teamcode.opmode.roverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Encoder Tester (Hinge,arm,tank)")
public class RuckusEncoderTest extends RuckusTankBasic {
    @Override public void init()
    {
        super.init();
        for (RuckusMotorName m : winchMotors)
        {
            m.activate();
        }
        for (RuckusMotorName m : armMotor)
        {
            m.activate();
            setMotorType(m.getMotorName(),DcMotor.RunMode.RUN_USING_ENCODER);
            setPower(m.getMotorName(),1.0);
        }
        for (RuckusServoName m : armSlideServo)
        {
            m.activate();
        }
        for (RuckusMotorName m : winchMotors)
        {
            m.activate();
        }

    }

    @Override
    public void loop()
    {
        super.loop();
        setHingePower(gamepad1.left_trigger - gamepad1.right_trigger);
        //setArmPower(gamepad2.left_trigger-gamepad2.right_trigger);
        incrementArmTargetPositionWithEncoder(gamepad2.left_trigger-gamepad2.right_trigger);
        slideArm((gamepad2.left_bumper ? 1 : 0) + (gamepad2.right_bumper ? -1 : 0));
        telemetry.addData("Arm Position", getMotorPosition(RuckusMotorName.MAIN_ARM.getMotorName()));
        telemetry.addData("Arm Target Position", getMotorTargetPosition(RuckusMotorName.MAIN_ARM.getMotorName()));

    }
    private boolean position;
    private int position1;
    private int position2;
}
