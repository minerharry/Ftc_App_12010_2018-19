package org.firstinspires.ftc.teamcode.opmode.roverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="SamplingFromLatch")
public class SamplingFromLatch extends RuckusStateMachineAuto {

    public void init()
    {
        route = AutonomousRoute.Sample_From_Lander;
        super.init();
    }

    public void loop()
    {
        super.loop();
    }
}
