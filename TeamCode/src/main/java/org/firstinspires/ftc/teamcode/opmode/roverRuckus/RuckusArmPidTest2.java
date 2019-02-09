package org.firstinspires.ftc.teamcode.opmode.roverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Arm Pid Tester 2")
public class RuckusArmPidTest2 extends RuckusTankBasic {
    @Override
    public void init()
    {
        super.init();
        for(RuckusMotorName motor : armMotor)
        {
            motor.activate();
        }
        setMotorType(armMotor[0].getMotorName(),DcMotor.RunMode.RUN_TO_POSITION);
        setPower(armMotor[0].getMotorName(),0.8);

    }
    @Override
    public void loop()
    {
        super.loop();
        setMotorTargetPosition(RuckusMotorName.MAIN_ARM.getMotorName(),
                getMotorTargetPosition(RuckusMotorName.MAIN_ARM.getMotorName())+
                        (int)(10*(gamepad2.left_trigger-gamepad2.right_trigger)));
        telemetry.addData("Arm Target Position", getMotorTargetPosition(RuckusMotorName.MAIN_ARM.getMotorName()));
        getMotorTargetPosition(RuckusMotorName.MAIN_ARM.getMotorName());


    }
}
