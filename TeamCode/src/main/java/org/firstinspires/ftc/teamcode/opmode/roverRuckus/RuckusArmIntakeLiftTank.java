package org.firstinspires.ftc.teamcode.opmode.roverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Ruckus Full Teleop")
public class RuckusArmIntakeLiftTank extends RuckusTankBasic {
    private static final int speedMultiplier = 30;
    private static final int armMultiplier = 20;
    private static int targetPosition = 0;
    @Override
    public void init()
    {
        for(RuckusMotorName name : armMotor)
        {
            name.activate();
        }
        for(RuckusMotorName name : linearSlideMotor)
        {
            name.activate();
        }
        for(RuckusMotorName name : intakeMotor)
        {
            name.activate();
        }
        for(RuckusServoName name : armSlideServo)
        {
            name.activate();
        }
        super.init();
        for(RuckusMotorName name : linearSlideMotor)
        {
            setMotorType(name, DcMotor.RunMode.RUN_TO_POSITION);
            setPower(name,1.0);
        }
        for(RuckusMotorName name : armMotor)
        {
            setPower(name,1.0);
            setMotorType(name, DcMotor.RunMode.RUN_TO_POSITION);
        }
        setMotorType(RuckusMotorName.MAIN_ARM, DcMotor.RunMode.RUN_TO_POSITION);
        setArmPower(1);
        armTargetPosition = getMotorTargetPosition(RuckusMotorName.MAIN_ARM);
        setMotorType(linearSlideMotor[0], DcMotor.RunMode.RUN_TO_POSITION);
        setPower(linearSlideMotor[0],1.0);
        targetPosition = getMotorPosition(linearSlideMotor[0]);
        targetPosition = (targetPosition < liftMin? liftMin: (targetPosition > liftMax? liftMax: targetPosition));

    }

    @Override
    public void loop()
    {

        float bumperPower = (gamepad2.left_bumper?0.8f:0)+(gamepad2.right_bumper?-0.8f:0);
        slideArm(bumperPower);
        super.loop();
        incrementArmTargetPosition((int)((gamepad2.left_trigger-gamepad2.right_trigger)*armMultiplier));
        setIntakeMotorPower(gamepad2.left_stick_y);
        double inputPower = (gamepad1.left_trigger - gamepad1.right_trigger);
        double shift = speedMultiplier*inputPower;
        telemetry.addData("Input Power", inputPower);
        telemetry.addData("Target Position Shift",shift);
        targetPosition += (int)shift;
        targetPosition = (targetPosition < liftMin? liftMin: (targetPosition > liftMax? liftMax: targetPosition));
        setMotorTargetPosition(linearSlideMotor[0],targetPosition);
        telemetry.addData("New Target",targetPosition);
        telemetry.addData("Motor Position",getMotorPosition(linearSlideMotor[0]));
        telemetry.addData("Motor Target Position",getMotorTargetPosition(linearSlideMotor[0]));

    }
    public void stop()
    {
        setMotorTargetPosition(linearSlideMotor[0],0);
    }
}
