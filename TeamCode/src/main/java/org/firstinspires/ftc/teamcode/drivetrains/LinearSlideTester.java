package org.firstinspires.ftc.teamcode.drivetrains;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Linear Slide Tester")
public class LinearSlideTester extends OpMode {
    Servo slideServo = null;
    Servo flipServo = null;
    double slideAngle = 0;
    double flipAngle = 0;
    @Override
    public void init() {
        slideServo = hardwareMap.get(Servo.class,"Slide_Servo");
        flipServo = hardwareMap.get(Servo.class,"Scoop_Servo");
    slideAngle = slideServo.getPosition();
    flipAngle = flipServo.getPosition();
    }

    @Override
    public void loop()
    {

    }
}
