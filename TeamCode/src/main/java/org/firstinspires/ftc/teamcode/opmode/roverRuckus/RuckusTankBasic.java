package org.firstinspires.ftc.teamcode.opmode.roverRuckus;

public class RuckusTankBasic extends RuckusRobotHardware {
    private static final float turnLimiter = 0.6f;
    @Override
    public void init()
    {
        super.init();
    }

    @Override
    public void loop()
    {
        setDriveForTankForTurn(gamepad1.left_stick_y,gamepad1.right_stick_y,turnLimiter);
    }
}
