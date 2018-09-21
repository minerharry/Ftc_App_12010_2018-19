package org.firstinspires.ftc.teamcode.vision;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Teleop extends OpMode {

    private DcMotor frontleftMotor = null;
    private DcMotor frontrightMotor = null;
    private DcMotor backrightMotor = null;
    private DcMotor backleftMotor = null;


    public void init() {
        frontleftMotor = hardwareMap.get(DcMotor.class, "Front_Left_Motor");
        frontrightMotor = hardwareMap.get(DcMotor.class, "Front_Right_Motor");
        backrightMotor = hardwareMap.get(DcMotor.class, "Back_Right_Motor");
        backleftMotor = hardwareMap.get(DcMotor.class, "Back_Left_Motor");


        frontleftMotor.setDirection(DcMotor.Direction.REVERSE);
        backleftMotor.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("Inittialized", "Yeehaw");
    }

    public void loop() {
        double leftPower;
        double rightPower;
        leftPower = gamepad1.left_stick_y;
        rightPower = gamepad1.right_stick_y;
        frontleftMotor.setPower(leftPower);
        backleftMotor.setPower(leftPower);
        frontrightMotor.setPower(rightPower);
        backrightMotor.setPower(rightPower);

    }
}

