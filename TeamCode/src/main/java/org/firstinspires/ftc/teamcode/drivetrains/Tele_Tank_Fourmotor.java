package org.firstinspires.ftc.teamcode.drivetrains;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//import com.qualcomm.hardware.motors.NeveRest40Gearmotor;

@TeleOp(name="Tank_Drive_Fourmotor", group="Iterative Opmode")
public class Tele_Tank_Fourmotor extends OpMode{

    private DcMotor leftFrontMotor = null;
    private DcMotor leftBackMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor rightBackMotor = null;
    double leftPower;
    double rightPower;
    private double turnLimiter = 0.6; //the minimum limiting value for when limiting turning

    @Override
    public void init()
    {

leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
leftBackMotor = hardwareMap.get(DcMotor.class, "leftBackMotor");
rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
rightBackMotor = hardwareMap.get(DcMotor.class, "rightBackMotor");

rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
rightBackMotor.setDirection(DcMotor.Direction.REVERSE);
telemetry.addData("Yeehaw", 1);
    }
@Override
    public void loop()
    {
leftPower = gamepad1.left_stick_y;
rightPower = gamepad1.right_stick_y;

double percentDiff = Math.abs(leftPower-rightPower);
telemetry.addData("percentDiff", percentDiff);
double percentLimiter = 1 - ((1-turnLimiter)/2 * percentDiff);
telemetry.addData("percentLimiter", percentLimiter);
leftPower *= percentLimiter;
rightPower *= percentLimiter;
telemetry.addData("leftPower",leftPower);
telemetry.addData("rightPower",leftPower);
leftFrontMotor.setPower(leftPower);
leftBackMotor.setPower(leftPower);
rightFrontMotor.setPower(rightPower);
rightBackMotor.setPower(rightPower);

    }

}
