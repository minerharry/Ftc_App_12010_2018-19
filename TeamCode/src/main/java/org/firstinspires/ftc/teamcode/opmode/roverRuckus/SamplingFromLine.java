package org.firstinspires.ftc.teamcode.opmode.roverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="SamplingFromLine")
public class SamplingFromLine extends RuckusStateMachineAuto {

    public void init()
    {
        route = AutonomousRoute.Sample_From_Line;
        super.init();
    }

    public void loop()
    {
        super.loop();
    }
}
    
