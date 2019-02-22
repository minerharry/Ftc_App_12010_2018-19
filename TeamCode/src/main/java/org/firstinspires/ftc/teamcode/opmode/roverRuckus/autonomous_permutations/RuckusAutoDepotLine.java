package org.firstinspires.ftc.teamcode.opmode.roverRuckus.autonomous_permutations;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.roverRuckus.RuckusStateMachineAuto;

@Autonomous(name="DepotLine")
public class RuckusAutoDepotLine extends RuckusStateMachineAuto {
    public void init()
    {
        route = AutonomousRoute.Depot_Route_Line;
        super.init();
    }
}
