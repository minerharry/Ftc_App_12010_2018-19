package org.firstinspires.ftc.teamcode.opmode.relicRecovery;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.control.Mecanum;
import org.firstinspires.ftc.teamcode.opmode.RobotHardware;

import java.util.ArrayList;

public abstract class RelicRobotHardware extends RobotHardware {
    public enum RelicMotorName{
        DRIVE_FRONT_LEFT ("Drive_Front_Left"),
        DRIVE_FRONT_RIGHT ("Drive_Front_Right"),
        DRIVE_BACK_LEFT ("Drive_Back_Left"),
        DRIVE_BACK_RIGHT ("Drive_Back_Right");

        private String myName;
        private MotorName myMotorName;
        RelicMotorName(String name) {
            myName = name;
            myMotorName = new RobotHardware.MotorName(name);
        }
        String getName()
        {
            return myName;
        }
        MotorName getMotorName()
        {
            return myMotorName;
        }

   }
   public enum RelicServoName
    {
        JEWEL_DROP ("Servo_Jewel_Drop"),
        JEWEL_HIT ("Servo_Jewel_Hit");
        private String myName;
        private ServoName myServoName;
        RelicServoName(String name)
        {
            myName = name;
            myServoName = new RobotHardware.ServoName(name);
        }
        String getName()
        {
            return myName;
        }
        ServoName getServoName()
        {
            return myServoName;
        }
    }
    public enum RelicColorSensorName
    {
        JEWEL ("Color_Sensor_Jewel");
        private String myName;
        private ColorSensorName mySensorName;
        RelicColorSensorName(String name)
        {
            myName = name;
            mySensorName = new RobotHardware.ColorSensorName(name);
        }
        String getName()
        {
            return myName;
        }
        ColorSensorName getColorSensorName()
        {
            return mySensorName;
        }
    }



    public void init()
    {
        super.init();
        raisedJewelAngle = getResourceInt(
                R.integer.raised_jewel_angle_percent) / 100.0;
        loweredJewelAngle = getResourceInt(
                R.integer.lowered_jewel_angle_percent) / 100.0;
        centerJewelAngle = getResourceInt(
                R.integer.center_jewel_angle_percent) / 100.0;
        forwardJewelAngle = getResourceInt(
                R.integer.forward_jewel_angle_percent) / 100.0;
        backwardJewelAngle = getResourceInt(
                R.integer.backward_jewel_angle_percent) / 100.0;
    }
    /**
     * Sets the drive chain power from Mecanum motion.
     * Maintains relative speeds when changing angles.
     * @param motion The desired Mecanum motion.
     */
   protected void setDriveForMecanum(Mecanum.Motion motion) {
        Mecanum.Wheels wheels = Mecanum.motionToWheels(motion);
        setPower(RelicMotorName.DRIVE_FRONT_LEFT.getMotorName(), wheels.frontLeft);
        setPower(RelicMotorName.DRIVE_FRONT_RIGHT.getMotorName(), wheels.frontRight);
        setPower(RelicMotorName.DRIVE_BACK_LEFT.getMotorName(), wheels.backLeft);
        setPower(RelicMotorName.DRIVE_BACK_RIGHT.getMotorName(), wheels.backRight);
    }
    /**
     * Sets the drive chain power from Mecanum motion.
     * Uses max power output while changing speeds at angle motions.
     * @param motion The desired Mecanum motion.
     */
   protected void setDriveForMecanumForSpeed(Mecanum.Motion motion) {
        Mecanum.Wheels wheels = Mecanum.motionToWheels(motion).scaleWheelPower(
                Math.sqrt(2));
        setPower(RelicMotorName.DRIVE_FRONT_LEFT.getMotorName(), wheels.frontLeft);
        setPower(RelicMotorName.DRIVE_FRONT_RIGHT.getMotorName(), wheels.frontRight);
        setPower(RelicMotorName.DRIVE_BACK_LEFT.getMotorName(), wheels.backLeft);
        setPower(RelicMotorName.DRIVE_BACK_RIGHT.getMotorName(), wheels.backRight);
    }
    // Raises the jewel arm.
    protected void raiseJewelArm() {
        setAngle(RelicServoName.JEWEL_DROP.getServoName(), raisedJewelAngle);
    }

    // Lowers the jewel arm.
    protected void lowerJewelArm() {
        setAngle(RelicServoName.JEWEL_DROP.getServoName(), loweredJewelAngle);
    }

    // Centers the jewel arm.
    protected void centerJewelArm() {
        setAngle(RelicServoName.JEWEL_HIT.getServoName(), centerJewelAngle);
    }

    // Moves the jewel arm forward.
    protected void forwardJewelArm() {
        setAngle(RelicServoName.JEWEL_HIT.getServoName(), forwardJewelAngle);
    }

    // Moves the jewel arm backward.
    protected void backwardJewelArm() {
        setAngle(RelicServoName.JEWEL_HIT.getServoName(), backwardJewelAngle);
    }
    // Per robot tuning parameters.
    private double raisedJewelAngle;
    private double loweredJewelAngle;
    private double centerJewelAngle;
    private double forwardJewelAngle;
    private double backwardJewelAngle;

    /*
     * Sets the drive chain power.
     * @param left The power for the left two motors.
     * @param right The power for the right two motors.
     */
   /*protected void setDriveForTank(double left, double right) {
        setPower(MotorName.DRIVE_FRONT_LEFT, left);
        setPower(MotorName.DRIVE_BACK_LEFT, left);
        setPower(MotorName.DRIVE_FRONT_RIGHT, right);
        setPower(MotorName.DRIVE_BACK_RIGHT, right);
    }*/

   //converts in-class enums of RelicColorSensorName, RelicMotorName, and RelicServoName
    //into the main version so that the superclass can access them
    @Override
    public ArrayList<ColorSensorName> getColorSensors() {
        ArrayList<ColorSensorName> names = new ArrayList<ColorSensorName>();
        for(RelicColorSensorName name : RelicColorSensorName.values())
        {
            names.add(name.getColorSensorName());
        }
        return names;
    }

    @Override
    public ArrayList<MotorName> getMotors() {
        ArrayList<MotorName> names = new ArrayList<MotorName>();
        for(RelicMotorName name : RelicMotorName.values())
        {
            names.add(name.getMotorName());
        }
        return names;
    }

    @Override
    public ArrayList<ServoName> getServos() {
        ArrayList<ServoName> names = new ArrayList<ServoName>();
        for(RelicServoName name : RelicServoName.values())
        {
            names.add(name.getServoName());
        }
        return names;
    }
}
