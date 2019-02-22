package org.firstinspires.ftc.teamcode.opmode.roverRuckus.autonomous_permutations;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.roverRuckus.RuckusStateMachineAuto;

@Autonomous(name="Unlatch Test")
public class UnlatchTesterAuto extends RuckusStateMachineAuto {
    public void init()
    {
        route = AutonomousRoute.Unlatch_To_Scanning_Position;
        super.init();
    }
}
