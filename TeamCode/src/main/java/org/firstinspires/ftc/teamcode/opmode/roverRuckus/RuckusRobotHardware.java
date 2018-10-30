package org.firstinspires.ftc.teamcode.opmode.roverRuckus;

import android.content.res.Resources;

import org.firstinspires.ftc.teamcode.opmode.RobotHardware;
import org.firstinspires.ftc.teamcode.R;


import java.util.ArrayList;

public abstract class RuckusRobotHardware extends RobotHardware {
<<<<<<< HEAD
    public enum RuckusMotorName{
<<<<<<< HEAD
        DRIVE_FRONT_LEFT (R.string.backLeft),
        DRIVE_FRONT_RIGHT (R.string.backRight),
        DRIVE_BACK_LEFT (R.string.frontLeft),
        DRIVE_BACK_RIGHT (R.string.frontRight),
        WINCH_MAIN(R.string.winchMotorMain),
        WINCH_ARM(R.string.winchMotorArm);
=======
    public enum RuckusMotorName {
        DRIVE_FRONT_LEFT(getResourceString(R.string.backLeft)),
        DRIVE_FRONT_RIGHT(getResourceString(R.string.backRight)),
        DRIVE_BACK_LEFT(getResourceString(R.string.frontLeft)),
        DRIVE_BACK_RIGHT(getResourceString(R.string.frontRight)),
        WINCH_MAIN(getResourceString(R.string.winchMotorMain)),
        WINCH_ARM(getResourceString(R.string.winchMotorArm));
>>>>>>> 42c2b4541921dc083a89598b4279ca3ab538b2d8
=======
        DRIVE_FRONT_LEFT (getResourceString(R.string.backLeft)),
        DRIVE_FRONT_RIGHT (getResourceString(R.string.backRight)),
        DRIVE_BACK_LEFT (getResourceString(R.string.frontLeft)),
        DRIVE_BACK_RIGHT (getResourceString(R.string.frontRight)),
        WINCH_MAIN(getResourceString(R.string.winchMotorMain)),
        WINCH_ARM(getResourceString(R.string.winchMotorArm));
>>>>>>> parent of 152bd23... Fixed XML problems and added sections to the ExampleRobotHardware class

        private String myName;
        private MotorName myMotorName;
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
        RuckusMotorName(String name) {
            myName = name;
            myMotorName = new RobotHardware.MotorName(name);
>>>>>>> parent of 152bd23... Fixed XML problems and added sections to the ExampleRobotHardware class
        }
        String getName()
        {
            return myName;
        }
        MotorName getMotorName()
        {
<<<<<<< HEAD
            if(myName == null)
            {
                throw new NullPointerException("Error: " + this.getDeclaringClass().toString() + " Exception - Name not initialized from XML, make sure initRobot[Object]s() method was called during init()");
            }

=======

        RuckusMotorName(String name) {
            myName = name;
            myMotorName = new RobotHardware.MotorName(name);
        }

        String getName() {
            return myName;
        }

        MotorName getMotorName() {
>>>>>>> 42c2b4541921dc083a89598b4279ca3ab538b2d8
=======
>>>>>>> parent of 152bd23... Fixed XML problems and added sections to the ExampleRobotHardware class
            return myMotorName;
        }

    }
<<<<<<< HEAD
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
<<<<<<< HEAD
            if(myName == null)
            {
                throw new NullPointerException("Error: " +this.getDeclaringClass().toString() + " Exception - Name not initialized from XML, make sure initRobot[Object]s() method was called during init()");
            }
=======

    public enum RuckusServoName {
        J("3");
        private String myName;
        private ServoName myServoName;

        RuckusServoName(String name) {
            myName = name;
            myServoName = new RobotHardware.ServoName(name);
        }

        String getName() {
>>>>>>> 42c2b4541921dc083a89598b4279ca3ab538b2d8
=======
>>>>>>> parent of 152bd23... Fixed XML problems and added sections to the ExampleRobotHardware class
            return myName;
        }
<<<<<<< HEAD
        ServoName getServoName()
        {
<<<<<<< HEAD
            if(myName == null)
            {
                throw new NullPointerException("Error: " +this.getDeclaringClass().toString() + " Exception - Name not initialized from XML, make sure initRobot[Object]s() method was called during init()");
            }
=======

        ServoName getServoName() {
>>>>>>> 42c2b4541921dc083a89598b4279ca3ab538b2d8
=======
>>>>>>> parent of 152bd23... Fixed XML problems and added sections to the ExampleRobotHardware class
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
<<<<<<< HEAD

    @Override
    public void init() {
        RuckusMotorName.initRobotMotors(hardwareMap);
        RuckusServoName.initRobotServos(hardwareMap);

        super.init();
    }
=======
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
>>>>>>> 42c2b4541921dc083a89598b4279ca3ab538b2d8
=======
    public void blah()
    {
        setPower(RuckusMotorName.DRIVE_BACK_LEFT.getMotorName(), 1.0);
    }

>>>>>>> parent of 152bd23... Fixed XML problems and added sections to the ExampleRobotHardware class
}
