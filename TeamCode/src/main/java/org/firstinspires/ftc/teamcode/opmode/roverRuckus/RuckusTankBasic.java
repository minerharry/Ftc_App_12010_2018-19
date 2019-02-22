package org.firstinspires.ftc.teamcode.opmode.roverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Basic Tank Drive")
public class RuckusTankBasic extends RuckusRobotHardware {
    private static final float turnLimiter = 0.6f;
    boolean tankDrive = true;
    boolean xPressed = false;
    private boolean yPressed = false;
    private boolean slowDrive = false;
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
        float powerPercent = (slowDrive ? 0.6f : 1.0f);
        if (tankDrive) {
            setDriveForTankForTurn(gamepad1.left_stick_y*powerPercent, -gamepad1.right_stick_y*powerPercent, turnLimiter);
        }
        else
        {
            setDriveForArcade(gamepad1.left_stick_x*powerPercent,gamepad1.left_stick_y*powerPercent);
        }
        for (RuckusMotorName name : RuckusRobotHardware.tankMotors)
        {
            telemetry.addData(name.getName() + " Position", getMotorPosition(name));
        }
        super.loop();
        if (gamepad1.y && !yPressed)
        {
            slowDrive = !slowDrive;
        }
        yPressed = gamepad1.y;
        if (gamepad1.x && !xPressed)
        {
            tankDrive = !tankDrive;
        }
        xPressed = gamepad1.x;
    }
}
