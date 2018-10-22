package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.base.Color;
import org.firstinspires.ftc.teamcode.vision.SimpleVuforia;

import java.util.ArrayList;


public abstract class AutonomousRobotHardware extends RobotHardware {

    // All gyros on the robot, in order of GyroName.
    private ArrayList<GyroSensor> allGyroSensors;

    // All color sensors on the robot, in order of ColorSensorName.
    private ArrayList<ColorSensor> allColorSensors;



    /**
     * Enables the use of Vuforia
     * @return Vuforia object
     */
    protected SimpleVuforia enableVuforia(){
        return new SimpleVuforia(getResourceString(R.string.vuforia_license_key));
    }


    // The color sensors on the robot.
    protected enum ColorSensorName {
        //Name of color sensors, won't need any this year. I think
    }

    /**
     * Gets the color value on the sensor.
     * @param sensor The sensor to read.
     * @param color The color channel to read intensity.
     */
    protected int getColorSensor(ColorSensorName sensor, Color.Channel color) {
        ColorSensor s = allColorSensors.get(sensor.ordinal());
        if (s == null) {
            telemetry.addData("Color Sensor Missing", sensor.name());
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
        ColorSensor s = allColorSensors.get(sensor.ordinal());
        if (s == null) {
            telemetry.addData("Color Sensor Missing", sensor.name());
        } else {
            s.enableLed(enabled);
        }
    }

    protected enum GyroSensorName {
        //Names of Gyro
    }

    /**
     * Calibrates the gyro
     * @param sensor The sensor to calibrate the gyro
     */
    protected void calibrateGyro(GyroSensorName sensor){
        GyroSensor s = allGyroSensors.get(sensor.ordinal());
        if (s == null) {
            telemetry.addData("Gyro Sensor Missing", sensor.name());
        } else {
            s.calibrate();
        }
    }

    /**
     * Checks if the gyro is still calibrating
     * @param sensor The sensor to check if the gyro is still calibrating
     * @return True for still calibrating and False for finished calibrating
     */
    protected boolean isCalibrating(GyroSensorName sensor){
        GyroSensor s = allGyroSensors.get(sensor.ordinal());
        if (s == null) {
            telemetry.addData("Gyro Sensor Missing", sensor.name());
            return false;
        } else {
            return s.isCalibrating();
        }
    }

    /**
     * Gets the heading of the gyro
     * @param sensor The sensor to find the heading of the gyro
     * @return An int that is the heading in degrees.
     */
    protected int getHeading(GyroSensorName sensor){
        GyroSensor s = allGyroSensors.get(sensor.ordinal());
        if (s == null) {
            telemetry.addData("Gyro Sensor Missing", sensor.name());
            return 0;
        } else {
            return s.getHeading();
        }
    }

    /**
     * Initialize the hardware handles.
     */
    @Override
    public void init() {
        super.init();
        allGyroSensors = new ArrayList<GyroSensor>();
        for (GyroSensorName g : GyroSensorName.values()) {
            try {
                allGyroSensors.add(hardwareMap.get(GyroSensor.class, g.name()));
            } catch (Exception e) {
                telemetry.addData("Motor Missing", g.name());
                allGyroSensors.add(null);
            }
        }

        allColorSensors = new ArrayList<ColorSensor>();
        for (ColorSensorName s : ColorSensorName.values()) {
            try {
                allColorSensors.add(hardwareMap.get(ColorSensor.class,
                        s.name()));
            } catch (Exception e) {
                telemetry.addData("Color Sensor Missing", s.name());
                allColorSensors.add(null);
            }
        }
    }

    @Override
    public void loop() {}

    @Override
    public void stop() {
        super.stop();

        for (ColorSensorName s : ColorSensorName.values()) {
            setColorSensorLedEnabled(s, false);
        }
    }
}
