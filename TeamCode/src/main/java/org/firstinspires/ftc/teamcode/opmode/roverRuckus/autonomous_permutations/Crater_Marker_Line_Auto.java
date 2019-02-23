package org.firstinspires.ftc.teamcode.opmode.roverRuckus.autonomous_permutations;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.roverRuckus.RuckusStateMachineAuto;

@Autonomous(name="Crater Marker Line Auto")
public class Crater_Marker_Line_Auto extends RuckusStateMachineAuto {
    public void init()
    {
        route = AutonomousRoute.Crater_Marker_Route_Line;
        super.init();
    }
}
