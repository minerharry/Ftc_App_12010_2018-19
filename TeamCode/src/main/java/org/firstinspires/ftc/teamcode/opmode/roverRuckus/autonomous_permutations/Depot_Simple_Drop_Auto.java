package org.firstinspires.ftc.teamcode.opmode.roverRuckus.autonomous_permutations;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.roverRuckus.RuckusStateMachineAuto;

@Autonomous(name="Depot Simple Drop Auto")
public class Depot_Simple_Drop_Auto extends RuckusStateMachineAuto {
    public void init()
    {
        route = AutonomousRoute.Depot_Simple_Route_Drop;
        super.init();
    }
}
