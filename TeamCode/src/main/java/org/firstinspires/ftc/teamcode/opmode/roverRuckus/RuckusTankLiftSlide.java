package org.firstinspires.ftc.teamcode.opmode.roverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="RuckusTankLift")
public class RuckusTankLiftSlide extends RuckusTankBasic {
    private static final int minEncoder = 25;
    private static final int maxEncoder = 26200;
    private static int targetPosition = 0;
    private static int speedMultiplier = 30;
    @Override
    public void init()
    {
        for (RuckusMotorName name : RuckusRobotHardware.linearSlideMotor)
        {
            name.activate();
        }
        super.init();
        setMotorType(linearSlideMotor[0], DcMotor.RunMode.RUN_TO_POSITION);
        setPower(linearSlideMotor[0],1.0);
        targetPosition = getMotorTargetPosition(linearSlideMotor[0]);
    }
    public void loop()
    {
super.loop();

        double inputPower = (gamepad2.left_trigger - gamepad2.right_trigger);
        double shift = speedMultiplier*inputPower;
        telemetry.addData("Input Power", inputPower);
        telemetry.addData("Target Position Shift",shift);
        targetPosition += (int)shift;
        targetPosition = (targetPosition < minEncoder? minEncoder : (targetPosition > maxEncoder ? maxEncoder : targetPosition));
        setMotorTargetPosition(linearSlideMotor[0],targetPosition);
        telemetry.addData("New Target",targetPosition);
        telemetry.addData("Motor Position",getMotorPosition(linearSlideMotor[0]));
        telemetry.addData("Motor Target Position",getMotorTargetPosition(linearSlideMotor[0]));
    }

}
