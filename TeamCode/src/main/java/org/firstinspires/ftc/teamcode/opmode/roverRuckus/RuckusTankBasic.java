package org.firstinspires.ftc.teamcode.opmode.roverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Basic Tank Drive")
public class RuckusTankBasic extends RuckusRobotHardware {
    private static final float turnLimiter = 0.6f;
    boolean tankDrive = true;
    boolean xPressed = false;
    @Override
    public void init() {
        for (RuckusMotorName m : tankMotors)
        {
            m.activate();
        }
        super.init();

//2230: distance to left mineral;
//1800: distance to middle mineral;
    }

    @Override
    public void loop()
    {
        if (tankDrive) {
            setDriveForTankForTurn(gamepad1.left_stick_y, -gamepad1.right_stick_y, turnLimiter);
        }
        else
        {
            setDriveForArcade(gamepad1.left_stick_x,gamepad1.left_stick_y);
        }
        for (RuckusMotorName name : RuckusRobotHardware.tankMotors)
        {
            telemetry.addData(name.getName() + " Position", getMotorPosition(name.getMotorName()));
        }
        super.loop();
        if (gamepad1.x && !xPressed)
        {
            tankDrive = !tankDrive;
        }
        xPressed = gamepad1.x;
    }
}
