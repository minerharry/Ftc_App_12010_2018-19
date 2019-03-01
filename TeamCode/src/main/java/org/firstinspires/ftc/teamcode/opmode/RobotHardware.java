package org.firstinspires.ftc.teamcode.opmode;

import android.content.res.Resources;
import android.util.TypedValue;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
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

    protected enum StartingColor {
        BLUE,
        RED,
    }

    // Get constant from resource file.
    protected int getResourceInt(int id) {
        return hardwareMap.appContext.getResources().getInteger(id);
    }

    // Get constant from resource file.
    protected double getResourceDouble(int id) {
        TypedValue outValue = new TypedValue();
        hardwareMap.appContext.getResources().getValue(id, outValue, true);
        return outValue.getFloat();
    }

    protected String getResourceString(int id) {
        return hardwareMap.appContext.getResources().getString(id);
    }

    public static interface MotorName {


        public String getName();


    }

    public static interface GyroName {

        public String getName();
    }


    /**
     * Initializes a BNO055IMU with set parameters
     * @param gyro the gyro to initialize
     * @param parameters parameters with which to init the IMU
     */
    protected void initGyroParameters(GyroName gyro, BNO055IMU.Parameters parameters)
    {
        if (allGyros == null)
        {
            telemetry.addData("Error:", "Attempted to set Gyro " + gyro.getName() + " Parameters before activated");
            return;
        }
        BNO055IMU g = allGyros.get(gyro);
        if (g == null) {
            telemetry.addData("Gyro Missing", gyro.getName());
        } else {
            g.initialize(parameters);
        }
        waitForStartTelemetry();
    }


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

    /**
     * Makes a motor whose mode is runToPosition run to some position
     * @param motor the motor to modify - should be set to runToPosition;
     * @param position the position in encoder ticks
     */
    protected void setMotorTargetPosition(MotorName motor, int position)
    {
        DcMotor m = allMotors.get(motor);
        if (m == null)
        {
            telemetry.addData("Motor Missing", motor.getName());
        }
        else if (m.getMode() != DcMotor.RunMode.RUN_TO_POSITION)
        {
            telemetry.addData("Motor not in mode run to Position:", motor.getName());
        }
        else
        {
            m.setTargetPosition(position);
        }
    }

    /**
     * Rotates motor some number of encoder ticks from its current target position
     * @param motor the motor to modify - needs to be in mode runToPosition
     * @param increment increment in encoder ticks
     */
    protected void incrementMotorToPosition(MotorName motor, int increment)
    {
        DcMotor m = allMotors.get(motor);
        if (m == null)
        {
            telemetry.addData("Motor Missing", motor.getName());
        }
        else if (m.getMode() != DcMotor.RunMode.RUN_TO_POSITION)
        {
            telemetry.addData("Motor mode not Run to Position:", motor.getName());
        }
        else
        {
            if (m.getPower() == 0)
            {
                telemetry.addData("Warning: Motor power equal to zero:", motor.getName());
            }
            m.setTargetPosition(m.getTargetPosition() + increment);
        }
    }

    /**
     * Returns the current position of a motor, in encoder ticks
     * @param motor the motor whose position to get
     **/
    protected int getMotorPosition(MotorName motor)
    {
        DcMotor m = allMotors.get(motor);
        if (m == null)
        {
            telemetry.addData("Motor Missing", motor.getName());
        }
        else
        {
            return m.getCurrentPosition();
        }
        return -1;
    }

    /**
     * Returns the current position of a motor, in encoder ticks
     * @param motor the motor whose position to get
     */
    protected int getMotorTargetPosition(MotorName motor)
    {
        DcMotor m = allMotors.get(motor);
        if (m == null)
        {
            telemetry.addData("Motor Missing", motor.getName());
        }
        else if (m.getMode() != DcMotor.RunMode.RUN_TO_POSITION && m.getMode() != DcMotor.RunMode.RUN_USING_ENCODER)
        {
            telemetry.addData("Motor mode not Run to Position:", motor.getName());
        }
        else
        {
            return m.getTargetPosition();
        }
        return -1;
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

    public static class CRServoName {
        private String myName;

        public CRServoName(String name) {myName = name;}

        public String getName() {return  myName;}
    }


    /**
     * sets the run mode of the specified motor
     * @param motor - the motor whose runmode is to be set
     * @param motorType - the mode to which to set the motor
     */
    protected void setMotorType(MotorName motor, DcMotor.RunMode motorType){
        DcMotor m = allMotors.get(motor);
        if (m == null) {
            telemetry.addData("Motor Missing", motor.getName());
        } else {
            m.setMode(motorType);
        }
    }


    /**
     * Calculates the countsPerInch of a motor
     * @param counts - the encoder ticks per revolution
     * @param gear - Motor gearing up - 60:1, 20:1, 40:1
     * @param diameter - diameter of wheel (inches
     */
    protected void setCountsPerInch(double counts, double gear, double diameter ){
        countsPerInch = (counts * gear) / (diameter * Math.PI);
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

    protected double getServoPosition(ServoName servo)
    {
        Servo s = allServos.get(servo);
        if (s == null) {
            telemetry.addData("Servo Missing", servo.getName());
            return 0.5;
        } else {
            return s.getPosition();
        }

    }

    /**
     * Sets the power of a continuous rotation servo
     * @param crServo the servo to modify
     * @param power the power to set [-1,1]
     */
    protected void setServoPower(CRServoName crServo, double power)
    {
        CRServo crs = allCRServos.get(crServo);
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
                telemetry.addData("Motor Missing from getMotors", m.getName());
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

        telemetry.addData("Gyro Status:", "Adding");
        allGyros = new HashMap<GyroName,BNO055IMU>();
        for (GyroName s : getGyros()) {
            try {
                telemetry.addData("Gyro Status:", "Name: " + s.getName());

                allGyros.put(s,hardwareMap.get(BNO055IMU.class,
                        s.getName()));

            } catch (Exception e) {
                telemetry.addData("Gyro Missing", s.getName());
                allGyros.put(s,null);
            }
        }

        allCRServos = new HashMap<CRServoName,CRServo>();
        for (CRServoName s : getCRServos()) {
            try {
                allCRServos.put(s, hardwareMap.get(CRServo.class, s.getName()));
            } catch (Exception e) {
                telemetry.addData("CRServo Missing", s.getName());
                allCRServos.put(s,null);
            }

        }


      /*  raiseJewelArm();
        centerJewelArm();*/
    }

    //return configuration-specific list of motors
    public ArrayList<MotorName> getMotors() {
        return new ArrayList<MotorName>();
    }

    //return configuration-specific list of motors
    public ArrayList<GyroName> getGyros() {
        return new ArrayList<GyroName>();
    }

    //return configuration-specific list of servos
    public ArrayList<ServoName> getServos() {
        return new ArrayList<ServoName>();
    }

    //return configuration-specific list of continuous rotation servos
    public ArrayList<CRServoName> getCRServos() { return new ArrayList<CRServoName>();}

    //return configuration-specific list of color sensors
    public ArrayList<ColorSensorName> getColorSensors() {
        return new ArrayList<ColorSensorName>();

    }

    public void loop() {
        if (!isStarted)
        {
            isStarted = true;
        }
    }

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

    /**
     * A function to directly get the gyro object from the robot. Different from other hardware
     * represented in this class because it is more complicated to mess with and benefits from easier
     * control over the BNO055IMU, but ideally all of those will have methods in this class
     * @param name
     * @return the imu from memory
     */
    protected BNO055IMU getGyro(GyroName name)
    {
        boolean broken = false;
        if (allGyros == null)
        {
            telemetry.addData("Error:",  "All Gyrps == null");
            broken = true;
        }
        if (name == null)
        {
            telemetry.addData("Error:","GyroName == null");
            broken = true;
        }
        if (!broken) {
            BNO055IMU imu = allGyros.get(name);
            if (imu == null) {
                telemetry.addData("Gyro Missing:", name.getName());
                return null;
            } else
                return imu;
        }
        return null;
    }
    public synchronized void waitForStartTelemetry() {
        while (!isStarted) {
            synchronized (this) {
                try {
                    this.wait();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
                telemetry.addData("Wait Status","Waiting for start");
            }
        }
    }
    // All motors on the robot, in order of MotorName.
    private Map<MotorName,DcMotor> allMotors;
    // All motors on the robot, in order of MotorName.
    private Map<GyroName, BNO055IMU> allGyros;
    // All servos on the robot, in order of ServoName.
    private Map<ServoName,Servo> allServos;
    // All CRServos on the robot, in order of ServoName.
    private Map<CRServoName, CRServo> allCRServos;
    // All color sensors on the robot, in order of ColorSensorName.
    private Map<ColorSensorName,ColorSensor> allColorSensors;


    //whether the opMode has started
    private boolean isStarted = false;




}
