package org.firstinspires.ftc.teamcode.drivetrains;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Drivetrain extends OpMode {

    private DcMotor frontLeftMotor= null;
    private DcMotor frontRightMotor= null;
    private DcMotor backRightMotor= null;
    private DcMotor backLeftMotor= null;


    public void init (){
        frontLeftMotor = hardwareMap.get (DcMotor.class,"frontLeftMotor");
        frontRightMotor= hardwareMap.get (DcMotor.class,"frontRightMotor");
        backRightMotor = hardwareMap.get (DcMotor.class,"backRightMotor");
        backLeftMotor = hardwareMap.get  (DcMotor.class,"backLeftMotor");
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
                telemetry.addData("Initialized", "Yeehaw");
    }
    public void loop(){
        double leftPower;
        double rightPower;
        leftPower=gamepad1.left_stick_y;
        rightPower=gamepad1.right_stick_y;
        frontLeftMotor.setPower(leftPower);
        backLeftMotor.setPower(leftPower);
        frontRightMotor.setPower(rightPower);
        backRightMotor.setPower(rightPower);

    }
}
