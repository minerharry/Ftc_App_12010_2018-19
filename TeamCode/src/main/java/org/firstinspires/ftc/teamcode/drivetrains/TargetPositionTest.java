package org.firstinspires.ftc.teamcode.drivetrains;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="TargetPositionTester")
public class TargetPositionTest extends OpMode {
    private int encoderRatio = 1;
    private DcMotor encoderMotor = null;
    private String motorname = "Motor_Arm";
    @Override
    public void init() {
        encoderMotor = hardwareMap.get(DcMotor.class,motorname);
        encoderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void loop() {
        telemetry.addData("Motor Position:", encoderMotor.getCurrentPosition());
        telemetry.addData("Motor Target Position:", encoderMotor.getTargetPosition());
        telemetry.addData("Encoder Ratio: ", encoderRatio);
        int currentTargetPosition = encoderMotor.getTargetPosition();
        encoderMotor.setTargetPosition(currentTargetPosition + encoderRatio*Math.round(gamepad1.left_stick_y));
        encoderRatio += Math.round(10*gamepad1.right_stick_y);
        encoderMotor.setPower(1.0);
    }
}
