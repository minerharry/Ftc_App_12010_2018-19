package org.firstinspires.ftc.teamcode.opmode.roverRuckus;

public class RuckusDropAuto extends RuckusRobotHardware {
    private double dropTime=1;
    private boolean hasStarted;
    private double startTime;
    public void init()
    {
        activateAll();
        super.init();
        resetStartTime();
    }
    public void loop()
    {
        if (!hasStarted)
        {
            resetStartTime();
            startTime = getRuntime();
        }
        if (getRuntime()-startTime < dropTime)
        {
            setHingePower(0.5f);
        }


    }
}
