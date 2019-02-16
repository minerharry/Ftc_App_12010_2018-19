package org.firstinspires.ftc.teamcode.opmode.roverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="DepotLine")
public class RuckusAutoDepotLine extends RuckusStateMachineAuto {
    public void init()
    {
        route = AutonomousRoute.Depot_Route_Line;
        super.init();
    }
}
