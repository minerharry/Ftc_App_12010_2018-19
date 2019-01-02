package org.firstinspires.ftc.teamcode.control;

/**
 * Created by Colin_Zhu on 2/10/2018.
 * Modified by Harrison Truscott over Dec 2018 - march 2019
 * File by Pmtischler
 */

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * PID Controller: kp * (e + (integral(e) / ti) + (td * derivative(e))).
 * https://en.wikipedia.org/wiki/PID_controller#Ideal_versus_standard_PID_form
 * Tutorial: https://pmtischler-ftc-app.readthedocs.io/en/latest/tutorials/pid_control.html
 * Help on tuning: Section called manual tuning
 * For more info on the bounding variables (intMin/intMax/outMin/outMax) see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-reset-windup/
 * tl;dr: if the integral term thinks its making more of a difference than it is because hardware has reached its limit, the integral will just increase and increase bc of the growing diff. IntMin and intMax are the limits of the integral;
 * basically the maximum possible effect in terms of the input. That is, the maximum end effect the PID can have on the input.
 * If your Pid is regulating speed, this would be something like a maximum speed the motor can turn on average.
 * Output clamping is purely the limits of the acceptable outputs; for something like motor speed, it's 1.0 and -1.0
 */
public class Pid {
    // Proportional factor to scale error to output.
    private double kp;
    // The number of seconds to eliminate all past errors.
    private double ti;
    // The number of seconds to predict the error in the future.
    private double td;
    // The min of the running integral.
    private double integralMin;
    // The max of the running integral.
    private double integralMax;
    // The min allowed PID output.
    private double outputMin;
    // The max allowed PID output.
    private double outputMax;
    // The last error value.
    private double previousError;
    // The discrete running integral (bounded by integralMax).
    private double runningIntegral;

    /**
     * Creates a PID Controller.
     *
     * @param kp          Proportional factor to scale error to output. -- reaction to the past errors; too high and it'll overreact, to low and it'll be sluggish
     * @param ti          The number of seconds to eliminate all past errors. -- for the integral
     * @param td          The number of seconds to predict the error in the future. -- for the derivative
     * @param integralMin The min of the running integral. -- see below
     * @param integralMax The max of the running integral. -- the min/maxes of the possible input
     * @param outputMin   The min of the PID output. -- see below
     * @param outputMax   The max of the PID output. -- the desired bounds of the output of the PID function (related to but NOT THE SAME AS Kp)
     */
    public Pid(double kp, double ti, double td, double integralMin,
               double integralMax, double outputMin, double outputMax) {
        setPidVariables(kp, ti, td, integralMin, integralMax, outputMin, outputMax);
    }

    public Pid(PIDConstants constants) {
        setPidVariables(constants.kp, constants.ti, constants.td, constants.intMin, constants.intMax, constants.outMin, constants.outMax);
    }

    /**
     * Clamps a value to a given range.
     *
     * @param value The value to clamp.
     * @param min   The min clamp.
     * @param max   The max clamp.
     * @return The clamped value.
     */
    public static double clampValue(double value, double min, double max) {
        return Math.min(max, Math.max(min, value));
    }

    public void setPidVariables(double kp, double ti, double td, double integralMin,
                                double integralMax, double outputMin, double outputMax) {
        this.kp = kp;
        this.ti = ti;
        this.td = td;
        this.integralMin = integralMin;
        this.integralMax = integralMax;
        this.outputMin = outputMin;
        this.outputMax = outputMax;

        this.previousError = 0;
        this.runningIntegral = 0;
    }

    /**
     * Performs a PID update and returns the output control.
     *
     * @param desiredValue The desired state value (e.g. speed).
     * @param actualValue  The actual state value (e.g. speed).
     * @param dt           The amount of time (sec) elapsed since last update.
     * @return The output which impacts state value (e.g. motor throttle).
     */
    public double update(double desiredValue, double actualValue, double dt) {
        double e = desiredValue - actualValue;
        runningIntegral = clampValue(runningIntegral + e * dt,
                integralMin, integralMax);
        double d = (e - previousError) / dt;
        double output = clampValue(kp * (e + (runningIntegral / ti) + (td * d)),
                outputMin, outputMax);

        previousError = e;
        return output;
    }


    public static class PIDConstants {
        public double kp;
        public double ti;
        public double td;
        public double intMin, intMax;
        public double outMin, outMax;

        public PIDConstants(double kp, double ti, double td, double intMin, double intMax, double outMin, double outMax)
        {
            this.kp = kp;
            this.ti = ti;
            this.td = td;
            this.intMin = intMin;
            this.intMax = intMax;
            this.outMin = outMin;
            this.outMax = outMax;
        }

    }

    /**
     * Simple template of above PIDConstants to make tuning more regular
     * Assumes motor takes input from -1 to 1
     * To limit motor max speed, use PIDConstants with custom outMin and outMax
     */
    public static class MotorPIDConstants extends PIDConstants{
        public MotorPIDConstants(double kp, double ti, double td, double intMin, double intMax)
        {
            super(kp,ti,td,intMin,intMax,-1.0,1.0);
        }

    }
    /**
     * Further template of above MotorPIDConstants to make tuning more regular
     * Assumes motor takes input from -1 to 1 and takes a maxspeed instead of intMin/max
     * To limit motor max speed, use PIDConstants with custom outMin and outMax
     * to use custom integral limiting,
     */
    public static class DrivePIDConstants extends MotorPIDConstants{
        public DrivePIDConstants(double kp, double ti, double td, double maxWheelSpeed)
        {
            super(kp,ti,td,-maxWheelSpeed,maxWheelSpeed);
        }

    }
}
