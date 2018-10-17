package org.firstinspires.ftc.teamcode.drivetrains;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="Hinge Mark 2 Tester w/ Adjustability", group="Testing")
public class HingeMk2Tester  extends OpMode {

    private DcMotor mainWinchMotor = null;
    private DcMotor secondaryLiftMotor = null;
    private float ratio = 1;

    private String mainMotorName = "Main_Motor";
    private String secondaryMotorName = "Secondary_Motor";

    @Override
    public void init()
    {
        mainWinchMotor = hardwareMap.get(DcMotor.class, mainMotorName);
        secondaryLiftMotor = hardwareMap.get(DcMotor.class,secondaryMotorName);
        telemetry.addData("Status", "Initialized");
    }
    @Override
    public void loop()
    {
        double mainPower = 1*gamepad1.left_stick_y;
        mainWinchMotor.setPower(mainPower);
        telemetry.addData("Main Power", mainPower);
        double secondaryPower = 0.2*gamepad1.right_stick_y;
        secondaryLiftMotor.setPower(secondaryPower);
        telemetry.addData("Secondary Power", secondaryPower);
    }

}
