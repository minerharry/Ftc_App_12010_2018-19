package org.firstinspires.ftc.teamcode.opmode.roverRuckus;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="Encoder+Gyro Debugging")
public class RuckusGyroEncoderTester extends RuckusTankBasic {
    //-22: left
    //0: middle:
    //27: right
    private double globalX;
    private BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    @Override
    public void init()
    {
        super.init();
        for (RuckusMotorName m : tankMotors)
        {
            setMotorType(m, DcMotor.RunMode.RUN_USING_ENCODER);
        }
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

    }
    public void loop()
    {
        super.loop();
        for(RuckusMotorName name : tankMotors)
        {
            telemetry.addData("Motor " + name.getName() + " Position", getMotorPosition(name));
        }
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float x  = angles.firstAngle;
        x = ( x < -180 ? x + 360 : (x > 180 ? x - 360 : x));
        double deltaX = lastAngles.firstAngle - x;
        deltaX = ( deltaX < -180 ? deltaX + 360 : (deltaX > 180 ? deltaX - 360 : deltaX));

            globalX = deltaX + globalX;

        telemetry.addData("My angle",globalX);
        lastAngles = angles;

    }
}
