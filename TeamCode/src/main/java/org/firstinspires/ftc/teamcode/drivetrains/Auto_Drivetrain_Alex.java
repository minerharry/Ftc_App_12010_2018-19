package org.firstinspires.ftc.teamcode.drivetrains;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Linear OpMode", group="Linear Opmode")
public class Auto_Drivetrain_Alex extends LinearOpMode {
    private DcMotor leftFrontMotor = null;
    private DcMotor leftBackMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor rightBackMotor = null;


    public synchronized void waitForStart()
    {
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        leftBackMotor = hardwareMap.get(DcMotor.class, "leftBackMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rightBackMotor");

        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("Yeehaw", 1);
        resetStartTime();
    }

    public void runOpMode()
    {
       waitForStart();
    while(opModeIsActive()) {
        double thisIsTheVariableForTheRunTimeThatHarrisonWantedMeToChangeToSomethingMoreAccurateWhichMakesSenseSinceItsWhatINormallyDoButIWasLazy = getRuntime();
        if (thisIsTheVariableForTheRunTimeThatHarrisonWantedMeToChangeToSomethingMoreAccurateWhichMakesSenseSinceItsWhatINormallyDoButIWasLazy <= 10) {


            leftFrontMotor.setPower(1);
            leftBackMotor.setPower(1);
            rightFrontMotor.setPower(1);
            rightBackMotor.setPower(1);
        } else {
            leftFrontMotor.setPower(0);
            leftBackMotor.setPower(0);
            rightFrontMotor.setPower(0);
            rightBackMotor.setPower(0);
        }
        idle();
    }

    }

}
