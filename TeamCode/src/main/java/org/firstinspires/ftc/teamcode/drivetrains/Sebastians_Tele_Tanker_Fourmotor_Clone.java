package org.firstinspires.ftc.teamcode.drivetrains;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//import com.qualcomm.hardware.motors.NeveRest40Gearmotor;

@TeleOp(name="Tank_Drive_Fourmotor_Clone", group="Iterative Opmode")
public class Sebastians_Tele_Tanker_Fourmotor_Clone extends OpMode{

    private DcMotor leftFrontMotor = null;
    private DcMotor leftBackMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor rightBackMotor = null;
    private DcMotor armMotor = null;
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
armMotor = hardwareMap.get(DcMotor.class, "armMotor");

rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
rightBackMotor.setDirection(DcMotor.Direction.REVERSE);
telemetry.addData("Huzzah", 1);
    }
@Override
    public void loop()
    {
leftPower = gamepad1.left_stick_y;
rightPower = gamepad1.right_stick_y;
float armPower = gamepad1.left_trigger-gamepad1.right_trigger;
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
armMotor.setPower(armPower);

    }

}
