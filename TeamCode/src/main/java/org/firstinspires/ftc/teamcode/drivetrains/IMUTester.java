package org.firstinspires.ftc.teamcode.drivetrains;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="Imu tester")
public class IMUTester extends OpMode {
    BNO055IMU imu;
    double globalX;
    double globalY;
    double globalZ;
    AxesOrder myOrder = AxesOrder.ZYX;
    AxesOrder[] myOrders = {AxesOrder.XYZ, AxesOrder.XZY, AxesOrder.ZXY, AxesOrder.ZYX, AxesOrder.YXZ, AxesOrder.YZX};
    int myOrderIndex = 0;
    boolean a = false;
    private boolean modulo;
    
    Orientation lastAngles = new Orientation();
    @Override
    public void init()
    {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

    }

    @Override
    public void loop()
    {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC,myOrder,AngleUnit.DEGREES);
        float x  = angles.firstAngle;
        float y = angles.secondAngle;
        float z = angles.thirdAngle;
        x = ( x < -180 ? x + 360 : (x > 180 ? x - 360 : x));
        y = ( y < -180 ? y + 360 : (y > 180 ? y - 360 : y));
        z = ( z < -180 ? z + 360 : (z > 180 ? z - 360 : z));
        double deltaX = lastAngles.firstAngle - x;
        double deltaY = lastAngles.secondAngle - y;
        double deltaZ = lastAngles.thirdAngle - z;
        deltaX = ( deltaX < -180 ? deltaX + 360 : (deltaX > 180 ? deltaX - 360 : deltaX));
        deltaY = ( deltaY < -180 ? deltaY + 360 : (deltaY > 180 ? deltaY - 360 : deltaY));
        deltaZ = ( deltaZ < -180 ? deltaZ + 360 : (deltaZ > 180 ? deltaZ - 360 : deltaZ));
        telemetry.addData(getAxesNameByAxesOrder(myOrder,1) + " angle",x);
        telemetry.addData(getAxesNameByAxesOrder(myOrder,2) + " angle",y);
        telemetry.addData(getAxesNameByAxesOrder(myOrder,3) + " angle",z);
        if (modulo) {
            globalX = (deltaX + globalX + 540) % 360 - 180;

            globalY = (deltaY + globalY + 540) % 360 - 180;
            globalZ = (deltaZ + globalZ + 540) % 360 - 180;
        }
        else {
            globalX = deltaX + globalX;
            globalY = deltaY + globalY;
            globalZ = deltaZ + globalZ;
        }
        telemetry.addData("Global " + getAxesNameByAxesOrder(myOrder,1) + " angle", globalX);
        telemetry.addData("Global " + getAxesNameByAxesOrder(myOrder,2) + " angle",globalY);
        telemetry.addData("Global " + getAxesNameByAxesOrder(myOrder,3) + " angle",globalZ);
        telemetry.addData("Axes Order", myOrder);
        lastAngles = angles;
        if (gamepad1.x)
        {
            modulo = true;
        }
        if (gamepad1.y)
        {
            modulo = false;
        }
        if (gamepad1.a && !a)
        {
            myOrderIndex = (myOrderIndex == 5 ? 0 : myOrderIndex + 1);
            myOrder = myOrders[myOrderIndex];
        }
    }
    public String getAxesNameByAxesOrder(AxesOrder order, int number) {
        String[] axesnames = {"x","y","z"};
        String[] result = new String[3];
        int[] axesOrder = new int[3];
        switch (order)
        {
            case XYZ: {
                int[] tempArray = {1, 2, 3};
                axesOrder = tempArray;
                break;
            }
            case XZY: {
                int[] tempArray = {1, 3, 2};
                axesOrder = tempArray;
                break;
            }
            case ZYX: {
                int[] tempArray = {3, 2, 1};
                axesOrder = tempArray;
                break;
            }
            case ZXY: {
                int[] tempArray = {3, 1, 2};
                axesOrder = tempArray;
                break;
            }
            case YZX: {
                int[] tempArray = {2, 3, 1};
                axesOrder = tempArray;
                break;
            }
            case YXZ: {
                int[] tempArray = {1, 3, 2};
                axesOrder = tempArray;
                break;
            }
        }
        return axesnames[axesOrder[number-1]-1];
    }

}
