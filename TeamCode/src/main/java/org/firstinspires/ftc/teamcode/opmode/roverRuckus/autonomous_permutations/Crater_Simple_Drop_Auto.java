package org.firstinspires.ftc.teamcode.opmode.roverRuckus.autonomous_permutations;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.roverRuckus.RuckusStateMachineAuto;

@Autonomous(name="Crater Simple Drop Auto")
public class Crater_Simple_Drop_Auto extends RuckusStateMachineAuto {
    public void init()
    {
        route = AutonomousRoute.Crater_Simple_Route_Drop;
        super.init();
    }
}
