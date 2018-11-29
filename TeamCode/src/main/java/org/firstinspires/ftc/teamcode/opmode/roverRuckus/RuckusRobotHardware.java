package org.firstinspires.ftc.teamcode.opmode.roverRuckus;

import android.content.res.Resources;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.opmode.RobotHardware;
import org.firstinspires.ftc.teamcode.R;


import java.util.ArrayList;

public abstract class RuckusRobotHardware extends RobotHardware {
    //The groupings of hardware parts on the robot
    protected static RuckusMotorName[] tankMotors = {RuckusMotorName.DRIVE_FRONT_LEFT, RuckusMotorName.DRIVE_BACK_RIGHT, RuckusMotorName.DRIVE_BACK_LEFT, RuckusMotorName.DRIVE_FRONT_RIGHT};
    protected static RuckusMotorName[] winchMotors = {RuckusMotorName.WINCH_MAIN, RuckusMotorName.WINCH_ARM};
    protected static RuckusMotorName[] armMotor = {RuckusMotorName.MAIN_ARM};
    protected static RuckusCRServoName[] intakeServos = {RuckusCRServoName.INTAKE_LEFT,RuckusCRServoName.INTAKE_RIGHT};
    protected static RuckusServoName[] armSlideServo = {RuckusServoName.ARM_SLIDE};


    public enum RuckusMotorName {
        DRIVE_FRONT_LEFT (R.string.frontLeft),
        DRIVE_FRONT_RIGHT (R.string.frontRight),
        DRIVE_BACK_LEFT (R.string.backLeft),
        DRIVE_BACK_RIGHT (R.string.backRight),
        WINCH_MAIN(R.string.winchMotorMain),
        WINCH_ARM(R.string.winchMotorArm),
        MAIN_ARM(R.string.armMotorMain);

        private int myNameID;
        private String myName;
        private MotorName myMotorName;
        private boolean isActivated = false;

        RuckusMotorName(int nameID) {
            myNameID = nameID;

        }
        public void initRobot(HardwareMap map)
        {
            myName = map.appContext.getResources().getString(myNameID);
            myMotorName = new RobotHardware.MotorName(myName);
        }
        public void activate()
        {
            isActivated = true;
        }
        public static void initRobotMotors(HardwareMap map)
        {
            for (RuckusMotorName name : RuckusMotorName.values())
            {
                name.initRobot(map);
            }
        }

        public boolean getActivated()
        {
            return isActivated;
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
    public enum RuckusCRServoName
    {
        INTAKE_LEFT(R.string.intakeServoLeft),
        INTAKE_RIGHT(R.string.intakeServoRight);
        private int myNameID;
        private String myName;
        private CRServoName myServoName;
        private boolean isActivated = false;
        RuckusCRServoName(int nameID)
        {
            myNameID = nameID;
        }
        public void initRobot(HardwareMap map) {
            myName = map.appContext.getResources().getString(myNameID);
            myServoName = new CRServoName(myName);
        }
        public boolean getActivated()
        {
            return isActivated;
        }
        public void activate()
        {
            isActivated = true;
        }
        String getName()
        {
            if(myName == null)
            {
                throw new NullPointerException("Error: " +this.getDeclaringClass().toString() + " Exception - Name not initialized from XML, make sure initRobot[Object]s() method was called during init()");
            }
            return myName;

        }
        CRServoName getCRServoName()
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

    public enum RuckusServoName
    {
        SCOOP(R.string.scoopServo,scoopMin,scoopMax),
        ARM_SLIDE(R.string.armServoSlide,slideMin,slideMax);
        private int myNameID;
        private double myMin;
        private double myMax;
        private double myPos;
        private String myName;
        private ServoName myServoName;
        private boolean isActivated = false;
        RuckusServoName(int nameID)
        {
            myNameID = nameID;
        }
        RuckusServoName(int nameID,double min, double max)
        {
            myNameID = nameID;
            myMin = min;
            myMax = max;
        }
        public void initRobot(HardwareMap map) {
            myName = map.appContext.getResources().getString(myNameID);
            myServoName = new ServoName(myName);
        }
        public boolean getActivated()
        {
            return isActivated;
        }
        public void activate()
        {
            isActivated = true;
        }
        double getMin()
        {
            return myMin;
        }
        double getMax()
        {
            return myMax;
        }
        double getPos()
        {
            return myPos;
        }
        protected void setPos(double pos)
        {
            myPos = pos;
        }
        protected double verifyAngle(double angle)
        {
            angle = Math.max(angle,myMin);
            angle = Math.min(angle,myMax);
            return angle;
        }

        protected double incrementPos(double increment)
        {
            myPos = verifyAngle(myPos+increment);
            return myPos;
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
            if (name.getActivated())
            names.add(name.getMotorName());
        }
        return names;
    }

    @Override
    public ArrayList<ServoName> getServos() {
        ArrayList<ServoName> names = new ArrayList<ServoName>();
        for(RuckusRobotHardware.RuckusServoName name : RuckusRobotHardware.RuckusServoName.values())
        {
            if (name.getActivated())
            names.add(name.getServoName());
        }
        return names;
    }

    @Override
    public ArrayList<CRServoName> getCRServos() {
        ArrayList<CRServoName> names = new ArrayList<CRServoName>();
        for(RuckusRobotHardware.RuckusCRServoName name : RuckusRobotHardware.RuckusCRServoName.values())
        {
            if (name.getActivated())
                names.add(name.getCRServoName());
        }
        return names;
    }


    @Override
    public void init() {
        RuckusMotorName.initRobotMotors(hardwareMap);
        RuckusServoName.initRobotServos(hardwareMap);
        RuckusCRServoName.initRobotServos(hardwareMap);
        super.init();
        for (RuckusServoName name : RuckusServoName.values())
        {
            if (name.isActivated)
            {
                name.setPos(getServoPosition(name.getServoName()));
            }
        }

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
        right *= percentLimiter;
        setPower(RuckusMotorName.DRIVE_BACK_LEFT.getMotorName(), left);
        setPower(RuckusMotorName.DRIVE_FRONT_LEFT.getMotorName(), left);
        setPower(RuckusMotorName.DRIVE_BACK_RIGHT.getMotorName(), right);
        setPower(RuckusMotorName.DRIVE_BACK_RIGHT.getMotorName(), right);
    }

    protected void setIntakeState(boolean state)
    {
        if (intakeContinuous)
        {
            telemetry.addData("Intake Error:", "State is continous, not on/off. Please check code.");
            return;
        }
        intakeState = state;
        if(intakeState)
        {
            setServoPower(RuckusCRServoName.INTAKE_LEFT.getCRServoName(), intakePower);
            setServoPower(RuckusCRServoName.INTAKE_RIGHT.getCRServoName(), -intakePower);
        }
        else {
            setServoPower(RuckusCRServoName.INTAKE_LEFT.getCRServoName(),0);
            setServoPower(RuckusCRServoName.INTAKE_RIGHT.getCRServoName(),0);
        }
    }

    protected void setIntakePower(float power)
    {
        if (!intakeContinuous)
        {
            telemetry.addData("Intake Error:", "State is on/off, not continuous. Please check code.");
            return;
        }
        setServoPower(RuckusCRServoName.INTAKE_LEFT.getCRServoName(), power);
        setServoPower(RuckusCRServoName.INTAKE_RIGHT.getCRServoName(), -power);

    }

    protected void setArmPower(float power)
    {
        setPower(RuckusMotorName.MAIN_ARM.getMotorName(),power);
    }

    protected void setHingePower(float power)
    {
        if (power > 0)
        {
            setPower(RuckusMotorName.WINCH_MAIN.getMotorName(), power * winchMainRaisePower);
            setPower(RuckusMotorName.WINCH_ARM.getMotorName(), power * winchArmRaisePower);
        }
        else
        {
            setPower(RuckusMotorName.WINCH_MAIN.getMotorName(), power * winchMainLowerPower);
            setPower(RuckusMotorName.WINCH_ARM.getMotorName(), power * winchArmLowerPower);
        }
    }

    protected void setIntakeType(boolean continuous)
    {
        intakeContinuous = continuous;
    }


    /** various position enums for autonomi **/

    public enum FieldPosition {
        POSITION_CRATER,
        POSITION_DEPOT
    }

    protected void activateAll()
    {
        for(RuckusMotorName m : RuckusMotorName.values())
        {
            m.activate();
        }
        for (RuckusCRServoName c : RuckusCRServoName.values())
        {
            c.activate();
        }
        for (RuckusServoName s :RuckusServoName.values())
        {
            s.activate();
        }
    }

    protected void incrementScoop(float increment)
    {
        setAngle(RuckusServoName.SCOOP.getServoName(),RuckusServoName.SCOOP.incrementPos(increment*0.075));

    }

    protected void slideArm(float power)
    {
        setAngle(RuckusServoName.SCOOP.getServoName(),RuckusServoName.SCOOP.incrementPos(power*0.05));
    }

    //whether the intake is state based or continuous
    protected boolean intakeContinuous = false;
    //current state of the intakeServos
    private boolean intakeState = false;

    //power of the intake at full
    private static double intakePower = 0.8;

    //powers of the winch's main and arm motors when raising/lowering
    private static double winchMainRaisePower = -1, winchArmRaisePower = 1, winchMainLowerPower = -1, winchArmLowerPower = 0.25;

    private static double scoopMin = 0.2, scoopMax = 0.8;
    private static double slideMin = 0.1, slideMax = 1.0;
}