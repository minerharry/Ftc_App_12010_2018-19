package org.firstinspires.ftc.teamcode.opmode.roverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

public class RuckusTankArmIntakeHinge extends RuckusTankBasic {
    private static int targetPosition = 0;
    private static int speedMultiplier = 30;
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

        setMotorType(linearSlideMotor[0], DcMotor.RunMode.RUN_TO_POSITION);
        setPower(linearSlideMotor[0],1.0);
        targetPosition = getMotorTargetPosition(linearSlideMotor[0]);
    }

    @Override
    public void loop()
    {
        super.loop();
        setHingePower(gamepad1.left_trigger - gamepad1.right_trigger);
        setArmPower(gamepad2.left_trigger - gamepad2.right_trigger);
        double inputPower = (gamepad2.left_trigger - gamepad2.right_trigger);
        double shift = speedMultiplier*inputPower;
        telemetry.addData("Input Power", inputPower);
        telemetry.addData("Target Position Shift",shift);
        targetPosition += (int)shift;
        targetPosition = (targetPosition < liftMin? liftMin : (targetPosition > liftMax ? liftMax : targetPosition));
        setMotorTargetPosition(linearSlideMotor[0],targetPosition);
        telemetry.addData("New Target",targetPosition);
        telemetry.addData("Motor Position",getMotorPosition(linearSlideMotor[0]));
        telemetry.addData("Motor Target Position",getMotorTargetPosition(linearSlideMotor[0]));
        setIntakePower(gamepad1.left_stick_y);
        incrementScoop(gamepad2.right_stick_y);
        slideArm((gamepad2.left_bumper ? 1 : 0) + (gamepad2.right_bumper ? -1 : 0));
    }
}
