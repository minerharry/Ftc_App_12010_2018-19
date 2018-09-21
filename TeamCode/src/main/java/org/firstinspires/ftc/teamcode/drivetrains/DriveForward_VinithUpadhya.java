package org.firstinspires.ftc.teamcode.drivetrains;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class DriveForward_VinithUpadhya extends LinearOpMode{

    private DcMotor leftBackMotor = null;
    private DcMotor leftFrontMotor = null;
    private DcMotor rightBackMotor = null;
    private DcMotor rightFrontMotor = null;

    // prevents any conflicts with public/private

    public synchronized void waitForStart() {
        leftBackMotor = hardwareMap.get(DcMotor.class, "leftBackMotor");
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rightBackMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");

        rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Pairs the variables to the actual motors. Right motors set to reverse?

    }
    public synchronized void runOpMode() {
        double runTime = getRuntime();
        System.out.print (runTime);
        if (runTime <= 10) {
            leftBackMotor.setPower(1);
            leftFrontMotor.setPower(1);
            rightBackMotor.setPower(1);
            rightFrontMotor.setPower(1);
        }

         else {
            idle();
        }

            // The robot's motors will be powered for 10 seconds.




    }}






