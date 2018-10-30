package org.firstinspires.ftc.teamcode.opmode.roverRuckus;

import android.content.res.Resources;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.opmode.RobotHardware;
import org.firstinspires.ftc.teamcode.R;


import java.util.ArrayList;

public abstract class RuckusRobotHardware extends RobotHardware {
    public enum RuckusMotorName{
        DRIVE_FRONT_LEFT (R.string.backLeft),
        DRIVE_FRONT_RIGHT (R.string.backRight),
        DRIVE_BACK_LEFT (R.string.frontLeft),
        DRIVE_BACK_RIGHT (R.string.frontRight),
        WINCH_MAIN(R.string.winchMotorMain),
        WINCH_ARM(R.string.winchMotorArm);

        private int myNameID;
        private String myName;
        private MotorName myMotorName;
        RuckusMotorName(int nameID) {
            myNameID = nameID;

        }
        public void initRobot(HardwareMap map)
        {
            myName = map.appContext.getResources().getString(myNameID);
            myMotorName = new RobotHardware.MotorName(myName);
        }
        public static void initRobotMotors(HardwareMap map)
        {
            for (RuckusMotorName name : RuckusMotorName.values())
            {
                name.initRobot(map);
            }
        }


        String getName()
        {
            if(myName == null)
            {
                throw new NullPointerException("Error: " + this.getDeclaringClass().toString() + " Exception - Name not initialized from XML, make sure initRobot[Object]s() method was called during init()");
            }
            return myName;
        }
        MotorName getMotorName()
        {
            if(myName == null)
            {
                throw new NullPointerException("Error: " + this.getDeclaringClass().toString() + " Exception - Name not initialized from XML, make sure initRobot[Object]s() method was called during init()");
            }

            return myMotorName;
        }

    }
    public enum RuckusServoName
    {
        J(R.string.intakeServoLeft);
        private int myNameID;
        private String myName;
        private ServoName myServoName;
        RuckusServoName(int nameID)
        {
            myNameID = nameID;
        }
        public void initRobot(HardwareMap map) {
            myName = map.appContext.getResources().getString(myNameID);
            myServoName = new ServoName(myName);
        }
        String getName()
        {
            if(myName == null)
            {
                throw new NullPointerException("Error: " +this.getDeclaringClass().toString() + " Exception - Name not initialized from XML, make sure initRobot[Object]s() method was called during init()");
            }
            return myName;

        }
        ServoName getServoName()
        {
            if(myName == null)
            {
                throw new NullPointerException("Error: " +this.getDeclaringClass().toString() + " Exception - Name not initialized from XML, make sure initRobot[Object]s() method was called during init()");
            }
            return myServoName;

        }
        public static void initRobotServos(HardwareMap map)
        {
            for(RuckusServoName name : RuckusServoName.values())
            {
                name.initRobot(map);
            }
        }
    }
    @Override
    public ArrayList<MotorName> getMotors() {
        ArrayList<MotorName> names = new ArrayList<MotorName>();
        for(RuckusRobotHardware.RuckusMotorName name : RuckusRobotHardware.RuckusMotorName.values())
        {
            names.add(name.getMotorName());
        }
        return names;
    }

    @Override
    public ArrayList<ServoName> getServos() {
        ArrayList<ServoName> names = new ArrayList<ServoName>();
        for(RuckusRobotHardware.RuckusServoName name : RuckusRobotHardware.RuckusServoName.values())
        {
            names.add(name.getServoName());
        }
        return names;
    }

    @Override
    public void init() {
        RuckusMotorName.initRobotMotors(hardwareMap);
        RuckusServoName.initRobotServos(hardwareMap);

        super.init();
    }

    protected void setDriveForTank(float left, float right)
    {
        setPower(RuckusMotorName.DRIVE_BACK_LEFT.getMotorName(), left);
        setPower(RuckusMotorName.DRIVE_FRONT_LEFT.getMotorName(), left);
        setPower(RuckusMotorName.DRIVE_BACK_RIGHT.getMotorName(), right);
        setPower(RuckusMotorName.DRIVE_BACK_RIGHT.getMotorName(), right);
    }

    protected void setDriveForTankForTurn(float left, float right,float turnLimiter)
    {
        double percentDiff = Math.abs(left-left);
        double percentLimiter = 1 - ((1-turnLimiter)/2 * percentDiff);
        left *= percentLimiter;
        left *= percentLimiter;
        setPower(RuckusMotorName.DRIVE_BACK_LEFT.getMotorName(), left);
        setPower(RuckusMotorName.DRIVE_FRONT_LEFT.getMotorName(), left);
        setPower(RuckusMotorName.DRIVE_BACK_RIGHT.getMotorName(), right);
        setPower(RuckusMotorName.DRIVE_BACK_RIGHT.getMotorName(), right);
    }
}