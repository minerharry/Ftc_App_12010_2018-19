package org.firstinspires.ftc.teamcode.opmode;

import android.util.TypedValue;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.base.Color;
import org.firstinspires.ftc.teamcode.control.Mecanum;

import java.util.ArrayList;

/**
 * Hardware Abstraction Layer for Robot.
 * Provides common variables and functions for the hardware.
 * The robot configuration in the app should match enum names.
 * Per-robot customization configured in SharedCode/src/main/res/values/.
 */
public abstract class RobotHardwareEnum extends OpMode {

    private static double countsPerMotorRev;
    private static double driveGearReduction;     // This is < 1.0 if geared UP
    private static double wheelDiameterInches;     // For figuring circumference
    private static double countsPerInch;

    // Get constant from resource file.
    int getResourceInt(int id) {
        return hardwareMap.appContext.getResources().getInteger(id);
    }

    // Get constant from resource file.
    double getResourceDouble(int id) {
        TypedValue outValue = new TypedValue();
        hardwareMap.appContext.getResources().getValue(id, outValue, true);
        return outValue.getFloat();
    }

    String getResourceString(int id) {
        return hardwareMap.appContext.getResources().getString(id);
    }

    // The motors on the robot.
    protected enum MotorName {
        DRIVE_FRONT_LEFT,
        DRIVE_FRONT_RIGHT,
        DRIVE_BACK_LEFT,
        DRIVE_BACK_RIGHT,
    }

    /**
     * Sets the power of the motor.
     * @param motor The motor to modify.
     * @param power The power to set [-1, 1].
     */
    protected void setPower(MotorName motor, double power) {
        DcMotor m = allMotors.get(motor.ordinal());
        if (m == null) {
            telemetry.addData("Motor Missing", motor.name());
        } else {
            m.setPower(power);
        }
    }

    /**
     * Sets the drive chain power.
     * @param left The power for the left two motors.
     * @param right The power for the right two motors.
     */
    protected void setDriveForTank(double left, double right) {
        setPower(MotorName.DRIVE_FRONT_LEFT, left);
        setPower(MotorName.DRIVE_BACK_LEFT, left);
        setPower(MotorName.DRIVE_FRONT_RIGHT, right);
        setPower(MotorName.DRIVE_BACK_RIGHT, right);
    }

    /**
     * Sets the drive chain power from Mecanum motion.
     * Maintains relative speeds when changing angles.
     * @param motion The desired Mecanum motion.
     */
    protected void setDriveForMecanum(Mecanum.Motion motion) {
        Mecanum.Wheels wheels = Mecanum.motionToWheels(motion);
        setPower(MotorName.DRIVE_FRONT_LEFT, wheels.frontLeft);
        setPower(MotorName.DRIVE_FRONT_RIGHT, wheels.frontRight);
        setPower(MotorName.DRIVE_BACK_LEFT, wheels.backLeft);
        setPower(MotorName.DRIVE_BACK_RIGHT, wheels.backRight);
    }

    /**
     * Sets the drive chain power from Mecanum motion.
     * Uses max power output while changing speeds at angle motions.
     * @param motion The desired Mecanum motion.
     */
    protected void setDriveForMecanumForSpeed(Mecanum.Motion motion) {
        Mecanum.Wheels wheels = Mecanum.motionToWheels(motion).scaleWheelPower(
                Math.sqrt(2));
        setPower(MotorName.DRIVE_FRONT_LEFT, wheels.frontLeft);
        setPower(MotorName.DRIVE_FRONT_RIGHT, wheels.frontRight);
        setPower(MotorName.DRIVE_BACK_LEFT, wheels.backLeft);
        setPower(MotorName.DRIVE_BACK_RIGHT, wheels.backRight);
    }

    protected void useEncoders(MotorName motor, DcMotor.RunMode encoderType){
        DcMotor m = allMotors.get(motor.ordinal());
        if (m == null) {
            telemetry.addData("Motor Missing", motor.name());
        } else {
            m.setMode(encoderType);
        }
    }

    protected void setCountsPerInch(double counts, double gear, double inches ){
        countsPerInch = (counts * gear) / (inches * Math.PI);
    }


    // The servos on the robot.
    protected enum ServoName {
        //names for servos
    }

    /**
     * Sets the angle of the servo.
     * @param servo The servo to modify.
     * @param position The angle to set [0, 1].
     */
    protected void setAngle(ServoName servo, double position) {
        Servo s = allServos.get(servo.ordinal());
        if (s == null) {
            telemetry.addData("Servo Missing", servo.name());
        } else {
            s.setPosition(position);
        }
    }

    // Possible starting positions.
    protected enum StartPosition {
        //Field Positions
        //One Latched
        //One on the ground
        //The other ground place
    }

    // Returns a string representation of the starting position.
    String getStartPositionName(Color.Ftc c, StartPosition p) {
        return c.name() + "-" + p.name();
    }

    /**
     * Initialize the hardware handles.
     */
    public void init() {

        allMotors = new ArrayList<DcMotor>();
        for (MotorName m : MotorName.values()) {
            try {
                allMotors.add(hardwareMap.get(DcMotor.class, m.name()));
            } catch (Exception e) {
                telemetry.addData("Motor Missing", m.name());
                allMotors.add(null);
            }
        }

        allServos = new ArrayList<Servo>();
        for (ServoName s : ServoName.values()) {
            try {
                allServos.add(hardwareMap.get(Servo.class, s.name()));
            } catch (Exception e) {
                telemetry.addData("Servo Missing", s.name());
                allServos.add(null);
            }
        }


    }

    public void loop() {}

    /**
     * End of match, stop all actuators.
     */
    public void stop() {
        super.stop();

        for (MotorName m : MotorName.values()) {
            setPower(m, 0);
        }
    }

    // All motors on the robot, in order of MotorName.
    private ArrayList<DcMotor> allMotors;
    // All servos on the robot, in order of ServoName.
    private ArrayList<Servo> allServos;

}
