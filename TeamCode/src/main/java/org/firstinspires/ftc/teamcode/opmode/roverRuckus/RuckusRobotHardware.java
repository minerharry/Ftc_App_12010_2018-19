package org.firstinspires.ftc.teamcode.opmode.roverRuckus;

import android.content.res.Resources;

import org.firstinspires.ftc.teamcode.opmode.RobotHardware;
import org.firstinspires.ftc.teamcode.R;


import java.util.ArrayList;

public abstract class RuckusRobotHardware extends RobotHardware {
    public enum RuckusMotorName{
        DRIVE_FRONT_LEFT (getResourceString(R.string.backLeft)),
        DRIVE_FRONT_RIGHT ("Drive_Front_Right"),
        DRIVE_BACK_LEFT ("Drive_Back_Left"),
        DRIVE_BACK_RIGHT ("Drive_Back_Right");


        private String myName;
        private MotorName myMotorName;
        RuckusMotorName(String name) {
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
    public enum RuckusServoName
    {
        J("3");
        private String myName;
        private ServoName myServoName;
        RuckusServoName(String name)
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
    @Override
    public ArrayList<MotorName> getMotors() {
        RuckusMotorName.
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
}
