package org.firstinspires.ftc.teamcode.opmode.roverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Arm Pid Tester")
public class RuckusArmPidTest extends RuckusTankBasic {
    @Override
    public void init()
    {
        super.init();
        for(RuckusMotorName motor : armMotor)
        {
            motor.activate();
        }
        enableMainArmMaintainPid();

    }
    @Override
    public void loop()
    {
        super.loop();
        setMotorMaintainPosition(RuckusMotorName.MAIN_ARM,
                RuckusMotorName.MAIN_ARM.getMotorTargetMaintainPosition()+
                        (int)(10*(gamepad2.left_trigger-gamepad2.right_trigger)));
        telemetry.addData("Arm Target Position", RuckusMotorName.MAIN_ARM.getMotorTargetMaintainPosition());
        boolean leftTrigger = gamepad1.left_trigger > 0.4;
        boolean rightTrigger = gamepad1.right_trigger > 0.4;
        double increment = (rightTrigger ? 0.01 : 0.05);
        increment *= (leftTrigger ? -1 : 1);
        if(gamepad1.a && !aPressed)
        {
            RuckusMotorName.MAIN_ARM.getMyMaintainPid().tuneConstants(0,increment,0);
        }
        else if (gamepad1.b && !bPressed) {
            RuckusMotorName.MAIN_ARM.getMyMaintainPid().tuneConstants(0,0,increment);
        } else if (gamepad1.x && !xPressed) {
            RuckusMotorName.MAIN_ARM.getMyMaintainPid().tuneConstants(increment, 0, 0);
        }
        aPressed = gamepad1.a;
        bPressed = gamepad1.b;
        xPressed = gamepad1.x;
        telemetry.addData("Detailed Mode",(rightTrigger ? "On" : "Old"));
        telemetry.addData("Direction", (leftTrigger ? "Decreasing" : "increasing"));
        telemetry.addData("Kp", RuckusMotorName.MAIN_ARM.getMyMaintainPid().getKp());
        telemetry.addData("Td",RuckusMotorName.MAIN_ARM.getMyMaintainPid().getTd());
        telemetry.addData("Ti",RuckusMotorName.MAIN_ARM.getMyMaintainPid().getTi());


    }
    boolean aPressed;
    boolean bPressed;
    boolean xPressed;

}
