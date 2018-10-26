package org.firstinspires.ftc.teamcode.opmode.roverRuckus;

import android.content.res.Resources;

import org.firstinspires.ftc.teamcode.opmode.RobotHardware;
import org.firstinspires.ftc.teamcode.R;


import java.util.ArrayList;

public abstract class RuckusRobotHardware extends RobotHardware {
    public enum RuckusMotorName {
        DRIVE_FRONT_LEFT(getResourceString(R.string.backLeft)),
        DRIVE_FRONT_RIGHT(getResourceString(R.string.backRight)),
        DRIVE_BACK_LEFT(getResourceString(R.string.frontLeft)),
        DRIVE_BACK_RIGHT(getResourceString(R.string.frontRight)),
        WINCH_MAIN(getResourceString(R.string.winchMotorMain)),
        WINCH_ARM(getResourceString(R.string.winchMotorArm));

        private String myName;
        private MotorName myMotorName;

        RuckusMotorName(String name) {
            myName = name;
            myMotorName = new RobotHardware.MotorName(name);
        }

        String getName() {
            return myName;
        }

        MotorName getMotorName() {
            return myMotorName;
        }

    }

    public enum RuckusServoName {
        J("3");
        private String myName;
        private ServoName myServoName;

        RuckusServoName(String name) {
            myName = name;
            myServoName = new RobotHardware.ServoName(name);
        }

        String getName() {
            return myName;
        }

        ServoName getServoName() {
            return myServoName;
        }
    }

    @Override
    public ArrayList<MotorName> getMotors() {
<<<<<<< HEAD
        RuckusMotorName.
                ArrayList<MotorName> names = new ArrayList<MotorName>();
        for (RuckusRobotHardware.RuckusMotorName name : RuckusRobotHardware.RuckusMotorName.values()) {
=======
        ArrayList<MotorName> names = new ArrayList<MotorName>();
        for(RuckusRobotHardware.RuckusMotorName name : RuckusRobotHardware.RuckusMotorName.values())
        {
>>>>>>> e98cec412e75aed7434e8a2dbd9d86208d33bc4e
            names.add(name.getMotorName());
        }
        return names;
    }

    @Override
    public ArrayList<ServoName> getServos() {
        ArrayList<ServoName> names = new ArrayList<ServoName>();
        for (RuckusRobotHardware.RuckusServoName name : RuckusRobotHardware.RuckusServoName.values()) {
            names.add(name.getServoName());
        }
        return names;
    }
<<<<<<< HEAD

    protected void setDriveForTank(double left, double right)
    {setPower(RuckusMotorName.DRIVE_BACK_LEFT.getMotorName(), left);
    setPower(RuckusMotorName.DRIVE_BACK_RIGHT.getMotorName(), right);
    setPower(RuckusMotorName.DRIVE_FRONT_LEFT.getMotorName(), left);
    setPower(RuckusMotorName.DRIVE_FRONT_RIGHT.getMotorName(), right);
    }
=======
    public void blah()
    {
        setPower(RuckusMotorName.DRIVE_BACK_LEFT.getMotorName(), 1.0);
    }

>>>>>>> e98cec412e75aed7434e8a2dbd9d86208d33bc4e
}
