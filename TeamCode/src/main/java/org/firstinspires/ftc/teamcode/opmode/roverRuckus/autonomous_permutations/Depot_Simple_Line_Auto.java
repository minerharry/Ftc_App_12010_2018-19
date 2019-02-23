package org.firstinspires.ftc.teamcode.opmode.roverRuckus.autonomous_permutations;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.roverRuckus.RuckusStateMachineAuto;

@Autonomous(name="Depot Simple Line Auto")
public class Depot_Simple_Line_Auto extends RuckusStateMachineAuto {
    public void init()
    {
        route = AutonomousRoute.Depot_Simple_Route_Line;
        super.init();
    }
}
