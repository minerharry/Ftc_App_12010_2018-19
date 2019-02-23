package org.firstinspires.ftc.teamcode.opmode.roverRuckus.autonomous_permutations;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.roverRuckus.RuckusStateMachineAuto;

@Autonomous(name="Depot Marker Drop Auto")
public class Depot_Marker_Drop_Auto extends RuckusStateMachineAuto {
    public void init()
    {
        route = AutonomousRoute.Depot_Route_Drop;
        super.init();
    }
}
