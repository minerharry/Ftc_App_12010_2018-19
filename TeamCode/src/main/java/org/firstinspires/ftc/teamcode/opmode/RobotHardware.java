package org.firstinspires.ftc.teamcode.opmode;

import android.content.res.Resources;
import android.util.TypedValue;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.base.Color;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

/**
 * Hardware Abstraction Layer for Robot.
 * Provides common variables and functions for the hardware.
 * The robot configuration in the app should match enum names.
 * Per-robot customization configured in SharedCode/src/main/res/values/.
 */
public abstract class RobotHardware extends OpMode {

    private static double countsPerMotorRev;
    private static double driveGearReduction;     // This is < 1.0 if geared UP
    private static double wheelDiameterInches;     // For figuring circumference
    private static double countsPerInch;

    // Get constant from resource file.
    protected int getResourceInt(int id) {
        return Resources.getSystem().getInteger(id);
    }

    // Get constant from resource file.
    protected double getResourceDouble(int id) {
        TypedValue outValue = new TypedValue();
        Resources.getSystem().getValue(id, outValue, true);
        return outValue.getFloat();
    }

    protected String getResourceString(int id) {
        return Resources.getSystem().getString(id);
    }

    public static class MotorName {
        private String myName;

        public MotorName(String name) {
            myName = name;
        }

        public String getName() {
            return myName;
        }
    }

    // The motors on the robot.
    /*protected enum MotorName {
        DRIVE_FRONT_LEFT,
        DRIVE_FRONT_RIGHT,
        DRIVE_BACK_LEFT,
        DRIVE_BACK_RIGHT,
    }*/

    /**
     * Sets the power of the motor.
     * @param motor The motor to modify.
     * @param power The power to set [-1, 1].
     */
    protected void setPower(MotorName motor, double power) {
        DcMotor m = allMotors.get(motor);
        if (m == null) {
            telemetry.addData("Motor Missing", motor.getName());
        } else {
            m.setPower(power);
        }
    }

    public static class ServoName {
        private String myName;

        public ServoName(String name)
        {
            myName = name;
        }

        public String getName()
        {
            return myName;
        }
    }


    protected void useEncoders(MotorName motor, DcMotor.RunMode encoderType){
        DcMotor m = allMotors.get(motor);
        if (m == null) {
            telemetry.addData("Motor Missing", motor.getName());
        } else {
            m.setMode(encoderType);
        }
    }

    protected void setCountsPerInch(double counts, double gear, double inches ){
        countsPerInch = (counts * gear) / (inches * Math.PI);
    }


  

    /**
     * Sets the angle of the servo.
     * @param servo The servo to modify.
     * @param position The angle to set [0, 1].
     */
    protected void setAngle(ServoName servo, double position) {
        Servo s = allServos.get(servo);
        if (s == null) {
            telemetry.addData("Servo Missing", servo.getName());
        } else {
            s.setPosition(position);
        }
    }


    public static class ColorSensorName {
        private String myName;
        public ColorSensorName(String name)
        {
            myName = name;
        }
        public String getName()
        {
            return myName;
        }
    }


    /**
     * Gets the color value on the sensor.
     * @param sensor The sensor to read.
     * @param color The color channel to read intensity.
     */
    protected int getColorSensor(ColorSensorName sensor, Color.Channel color) {
        ColorSensor s = allColorSensors.get(sensor);
        if (s == null) {
            telemetry.addData("Color Sensor Missing", sensor.getName());
            return 0;
        }

        switch (color) {
            case RED: return s.red();
            case GREEN: return s.green();
            case BLUE: return s.blue();
            case ALPHA: return s.alpha();
            default: return 0;
        }
    }

    /**
     * Sets the LED power for the color sensor.
     * @param sensor The sensor to set the LED power.
     * @param enabled Whether to turn the LED on.
     */
    protected void setColorSensorLedEnabled(ColorSensorName sensor,
                                         boolean enabled) {
        ColorSensor s = allColorSensors.get(sensor);
        if (s == null) {
            telemetry.addData("Color Sensor Missing", sensor.getName());
        } else {
            s.enableLed(enabled);
        }
    }




    /**
     * Initialize the hardware handles.
     */
    public void init() {



        allMotors = new HashMap<MotorName,DcMotor>();
        for (MotorName m : getMotors()) {

            try {
                allMotors.put(m,hardwareMap.get(DcMotor.class, m.getName()));
            } catch (Exception e) {
                telemetry.addData("Motor Missing", m.getName());
                allMotors.put(m,null);
            }
        }

        allServos = new HashMap<ServoName,Servo>();
        for (ServoName s : getServos()) {
            try {
                allServos.put(s,hardwareMap.get(Servo.class, s.getName()));
            } catch (Exception e) {
                telemetry.addData("Servo Missing", s.getName());
                allServos.put(s,null);
            }
        }

        allColorSensors = new HashMap<ColorSensorName,ColorSensor>();
        for (ColorSensorName s : getColorSensors()) {
            try {
                allColorSensors.put(s,hardwareMap.get(ColorSensor.class,
                                                    s.getName()));
            } catch (Exception e) {
                telemetry.addData("Color Sensor Missing", s.getName());
                allColorSensors.put(s,null);
            }
        }

      /*  raiseJewelArm();
        centerJewelArm();*/
    }

    //return configuration-specific list of motors
    public ArrayList<MotorName> getMotors() {
        return new ArrayList<MotorName>();
    }

    //return configuration-specific list of servos
    public ArrayList<ServoName> getServos() {
        return new ArrayList<ServoName>();
    }

    //return configuration-specific list of color sensors
    public ArrayList<ColorSensorName> getColorSensors() {
        return new ArrayList<ColorSensorName>();

    }

    public void loop() {}

    /**
     * End of match, stop all actuators.
     */
    public void stop() {
        super.stop();

        for (MotorName m : getMotors()) {
            setPower(m, 0);
        }

        for (ColorSensorName s : getColorSensors()) {
            setColorSensorLedEnabled(s, false);
        }

    }

    // All motors on the robot, in order of MotorName.
    private Map<MotorName,DcMotor> allMotors;
    // All servos on the robot, in order of ServoName.
    private Map<ServoName,Servo> allServos;
    // All color sensors on the robot, in order of ColorSensorName.
    private Map<ColorSensorName,ColorSensor> allColorSensors;




}
