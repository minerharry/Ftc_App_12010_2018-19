package org.firstinspires.ftc.teamcode.opmode.roverRuckus;

import android.content.res.Resources;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.control.Pid;
import org.firstinspires.ftc.teamcode.opmode.RobotHardware;
import org.firstinspires.ftc.teamcode.R;


import java.util.ArrayList;

/**
 * Created and implemented by Harrison Truscott of FTC Team 12010 during the 2018-19 season
 * For more details about game-specific robotHardware classes, see the hardwareExamples class
 * @see org.firstinspires.ftc.teamcode.opmode.hardwareExamples.ExampleRobotHardware
 */
public abstract class RuckusRobotHardware extends RobotHardware {
    //The groupings of hardware parts on the robot
    protected static RuckusMotorName[] tankMotors = {RuckusMotorName.DRIVE_FRONT_LEFT, RuckusMotorName.DRIVE_BACK_RIGHT, RuckusMotorName.DRIVE_BACK_LEFT, RuckusMotorName.DRIVE_FRONT_RIGHT};
    protected static RuckusMotorName[] winchMotors = {RuckusMotorName.WINCH_MAIN, RuckusMotorName.WINCH_ARM};
    protected static RuckusMotorName[] armMotor = {RuckusMotorName.MAIN_ARM};
    protected static RuckusCRServoName[] intakeServos = {RuckusCRServoName.INTAKE_LEFT,RuckusCRServoName.INTAKE_RIGHT};
    protected static RuckusServoName[] armSlideServo = {RuckusServoName.ARM_SLIDE};
    protected static RuckusMotorName[] linearSlideMotor = {RuckusMotorName.CLIMB_SLIDE};
    protected static RuckusMotorName[] intakeMotor = {RuckusMotorName.MOTOR_INTAKE};

    protected static RuckusRobotHardware instance;


    public enum RuckusMotorName implements MotorName{
        DRIVE_FRONT_LEFT (R.string.frontLeft),
        DRIVE_FRONT_RIGHT (R.string.frontRight),
        DRIVE_BACK_LEFT (R.string.backLeft),
        DRIVE_BACK_RIGHT (R.string.backRight),
        WINCH_MAIN(R.string.winchMotorMain),
        WINCH_ARM(R.string.winchMotorArm),
        MAIN_ARM(R.string.armMotorMain),
        CLIMB_SLIDE( R.string.linearSlide),
        MOTOR_INTAKE(R.string.motorIntake);

        private int myNameID; //The R.id for the motorName
        private String myName; //The name of the component; only defined after initRobot()
        //private MotorName myMotorName; //The MotorName that contains the name of the hardware piece
        private boolean isActivated = false; //whether the hardware is activated and will be initialized
        private boolean modeMaintainPos = false; //whether the motor will attempt to maintain its position
        private Pid myMaintainPid = null; // the PID used to maintain the arm at a certain point
        private int myMaintainTargetPosition; //the target at which the motor is to maintain its position
        private double pidLastUpdate = -1; //the last update of the PID

        RuckusMotorName(int nameID) {
            myNameID = nameID;
        }

        /**Generate the motor's name from xml using nameID*/
        public void initRobot(HardwareMap map)
        {
            myName = map.appContext.getResources().getString(myNameID);
           // myMotorName = new RobotHardware.MotorName(myName);
        }
        /**Specify that this motor is active and should be initialized from the hardwareMap to be used
        during runtime*/
        public void activate()
        {
            isActivated = true;
        }
        /**Initialize all of the motors' names from xml*/
        public static void initRobotMotors(HardwareMap map)
        {
            for (RuckusMotorName name : RuckusMotorName.values())
            {
                name.initRobot(map);
            }
        }
        /**Turn on the PID to maintain this motor's position*/
        public void activateMaintainPosition(Pid.PIDConstants constants, double startTime)
        {
            modeMaintainPos = true;
            myMaintainPid = new Pid(constants);
            pidLastUpdate = startTime;
        }
        /**Turn off the Maintain PID*/
        public void deactivateMaintainPosition()
        {
            modeMaintainPos = false;
            myMaintainPid = null;
            pidLastUpdate = -1;
        }
        /**Get the Maintain PID*/
        public Pid getMyMaintainPid()
        {
            return myMaintainPid;
        }
        /**Returns whether maintain is currently on*/
        public boolean getMaintainPositionActive()
        {
            return modeMaintainPos;
        }
        /**Update the maintain PID with the current position and the time*/
        public double updateMaintainPid(int currentPosition, double currentTime)
        {
            return myMaintainPid.update(myMaintainTargetPosition,currentPosition,currentTime-pidLastUpdate);
        }
        /**Set the maintain target position*/
        public void setMotorTargetPosition(int targetPosition)
        {
            myMaintainTargetPosition = targetPosition;
        }
        /**Returns the maintain target position*/
        public int getMotorTargetMaintainPosition()
        {
            return myMaintainTargetPosition;
        }
        /**Returns whether the motor is activated*/
        public boolean getActivated()
        {
            return isActivated;
        }
        /**Returns the string name associated with this motor's MotorName*/

        public String getName()
        {
            if(myName == null)
            {
                throw new NullPointerException("Error: " + this.getDeclaringClass().toString() + " Exception - Name not initialized from XML, make sure initRobot[Object]s() method was called during init()");
            }
            return myName;
        }
        /**Returns the motorName associated with this motor; called to interface with RobotHardware methods*/
        /*MotorName getMotorName()
        {
            if(myMotorName == null)
            {
                throw new NullPointerException("Error: " + this.getDeclaringClass().toString() + " Exception - Name not initialized from XML, make sure initRobot[Object]s() method was called during init()");
            }

            return myMotorName;
        }*/

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
        public static void initRobotCRServos(HardwareMap map)
        {
            for(RuckusCRServoName name : RuckusCRServoName.values())
            {
                name.initRobot(map);
            }
        }

    }

    public enum RuckusGyroName implements GyroName
    {
        HUB_2_IMU(R.string.hub2Imu);

        private int myNameID;
        private String myName;

        private boolean isActivated = false;
        private BNO055IMU.Parameters myParameters;
        RuckusGyroName(int nameID)
        {
            myNameID = nameID;
        }
        public void initRobot(HardwareMap map) {
            myName = map.appContext.getResources().getString(myNameID);

        }
        public boolean getActivated()
        {
            return isActivated;
        }
        public void activate()
        {
            isActivated = true;

        }
        public void setParameters(BNO055IMU.Parameters parameters)
        {
            if (!(myParameters == parameters)) {
                instance.initGyroParameters(this, parameters);
            }
            myParameters = parameters;
        }
        @Override
        public String getName()
        {
            if(myName == null)
            {
                throw new NullPointerException("Error: " +this.getDeclaringClass().toString() + " Exception - Name not initialized from XML, make sure initRobot[Object]s() method was called during init()");
            }
            return myName;

        }

        public static void initRobotGyros(HardwareMap map)
        {
            for(RuckusGyroName name : RuckusGyroName.values())
            {
                name.initRobot(map);
            }
        }

    }

    public enum RuckusServoName
    {
        SCOOP(R.string.scoopServo,scoopMin,scoopMax),
        ARM_SLIDE(R.string.armServoSlide,slideMin,slideMax);//NOTE: HI-TEC SERVO, Range only 0.5-1
        private int myNameID;
        private double myMin;
        private double myMax;
        private double myPos = 0.6;
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
            angle = (angle > myMax? myMax : (angle < myMin ? myMin : angle));
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
            names.add(name);
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
    public ArrayList<GyroName> getGyros() {
        ArrayList<GyroName> names = new ArrayList<GyroName>();
        for(RuckusRobotHardware.RuckusGyroName name : RuckusRobotHardware.RuckusGyroName.values())
        {
            if (name.getActivated())
                names.add(name);
        }
        telemetry.addData("GetGyros name:", names);
        return names;
    }


    public RuckusRobotHardware()
    {
        instance = this;
    }
    public void initXML()
    {
        telemetry.addData("XML","Initialized");
        RuckusMotorName.initRobotMotors(hardwareMap);
        RuckusServoName.initRobotServos(hardwareMap);
        RuckusCRServoName.initRobotCRServos(hardwareMap);
        RuckusGyroName.initRobotGyros(hardwareMap);
    }
    @Override
    public void init() {
        initXML();
        super.init();
        if (RuckusMotorName.CLIMB_SLIDE.getActivated())
        {
            liftTargetPosition = getMotorPosition(linearSlideMotor[0]);
            liftTargetPosition = (liftTargetPosition < liftMin? liftMin: (liftTargetPosition > liftMax? liftMax: liftTargetPosition));
        }
        for (RuckusServoName name : RuckusServoName.values())
        {
            if (name.isActivated)
            {
                //name.setPos(getServoPosition(name.getServoName()));
            }
        }

    }

    protected void setDriveForArcade(float x, float y)
    {
        double left = Range.clip(y-x,-1.0,1.0);
        double right = Range.clip(-x-y,-1.0,1.0);
        setPower(RuckusMotorName.DRIVE_BACK_LEFT, left);
        setPower(RuckusMotorName.DRIVE_FRONT_LEFT, left);
        setPower(RuckusMotorName.DRIVE_BACK_RIGHT, right);
        setPower(RuckusMotorName.DRIVE_FRONT_RIGHT, right);
    }
    protected void setDriveForTank(float left, float right)
    {
        setPower(RuckusMotorName.DRIVE_BACK_LEFT, left);
        setPower(RuckusMotorName.DRIVE_FRONT_LEFT, left);
        setPower(RuckusMotorName.DRIVE_BACK_RIGHT, right);
        setPower(RuckusMotorName.DRIVE_BACK_RIGHT, right);
    }

    protected void setDriveForTankForTurn(float left, float right,float turnLimiter)
    {
        double percentDiff = Math.abs(left-left);
        double percentLimiter = 1 - ((1-turnLimiter)/2 * percentDiff);
        left *= percentLimiter;
        right *= percentLimiter;
        setPower(RuckusMotorName.DRIVE_BACK_LEFT, left);
        setPower(RuckusMotorName.DRIVE_FRONT_LEFT, left);
        setPower(RuckusMotorName.DRIVE_BACK_RIGHT, right);
        setPower(RuckusMotorName.DRIVE_FRONT_RIGHT, right);
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

    protected void setIntakeMotorPower(float power)
    {
        setPower(RuckusMotorName.MOTOR_INTAKE,power);
    }

    protected void setArmPower(float power)
    {
        setPower(RuckusMotorName.MAIN_ARM,power);
    }

    protected void setHingePower(float power)
    {
        if (power < 0)
        {
            setPower(RuckusMotorName.WINCH_MAIN, -power * winchMainRaisePower);
            setPower(RuckusMotorName.WINCH_ARM, -power * winchArmRaisePower);
        }
        else
        {
            setPower(RuckusMotorName.WINCH_MAIN, power * winchMainLowerPower);
            setPower(RuckusMotorName.WINCH_ARM, power * winchArmLowerPower);
        }
    }

    protected void setIntakeType(boolean continuous)
    {
        intakeContinuous = continuous;
    }

    protected void slideLiftSlide(int ticks)
    {
        liftTargetPosition += ticks;
        liftTargetPosition = (liftTargetPosition < liftMin? liftMin: (liftTargetPosition > liftMax? liftMax: liftTargetPosition));
        setMotorTargetPosition(linearSlideMotor[0],liftTargetPosition);

    }

    protected void setLiftSlidePosition(int ticks)
    {
        liftTargetPosition = ticks;
        liftTargetPosition = (liftTargetPosition < liftMin? liftMin: (liftTargetPosition > liftMax? liftMax: liftTargetPosition));
        setMotorTargetPosition(linearSlideMotor[0],liftTargetPosition);

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
        telemetry.addData("Input Power", power);
        slidePos += power*0.004;
        slidePos = (slidePos > slideMax ? slideMax : (slidePos < slideMin ? slideMin : slidePos));
        setAngle(RuckusServoName.ARM_SLIDE.getServoName(),slidePos);
        telemetry.addData("Slide Pos",slidePos);


    }
    protected void incrementArmTargetPosition(int increment)
    {
        armTargetPosition += increment;
        armTargetPosition = (armTargetPosition < armMinPosition ? armMinPosition : (armTargetPosition > armMaxPosition ? armMaxPosition : armTargetPosition));
        setMotorTargetPosition(RuckusMotorName.MAIN_ARM,armTargetPosition);
    }
    protected void incrementArmTargetPositionWithEncoder(float power)
    {
        super.incrementMotorToPosition(RuckusMotorName.MAIN_ARM,Math.round( armIncrementRatio*power));
    }
    protected void enableMotorMaintainPosition(RuckusMotorName motorName, Pid.PIDConstants constants)
    {
        motorName.activateMaintainPosition(constants,time);
        setMotorType(motorName,DcMotor.RunMode.RUN_USING_ENCODER);
    }
    protected void disableMotorMaintainPosition(RuckusMotorName motorName)
    {
        motorName.deactivateMaintainPosition();
    }
    protected void updateMotorMaintainPosition(RuckusMotorName motor)
    {
        double power = (motor.updateMaintainPid(getMotorPosition(motor),time));
        telemetry.addData("Motor " + motor.getName() + " Pid Updated:", power);
        setPower(motor,power);

    }
    protected void setMotorMaintainPosition(RuckusMotorName motor, int newTargetPosition)
    {
        motor.setMotorTargetPosition(newTargetPosition);
    }
    protected void enableMainArmMaintainPid()
    {
        enableMotorMaintainPosition(RuckusMotorName.MAIN_ARM,ARM_PID_CONSANTS);
        telemetry.addData("Status", "Pid Initialized");
    }
    protected void raiseLiftToLatchingHeight()
    {
        setLiftSlidePosition(LIFT_ALIGN_HEIGHT);
    }
    protected void lowerLift()
    {
        setLiftSlidePosition(0);
    }

    @Override
    public void loop()
    {
        for(RuckusMotorName motor : RuckusMotorName.values())
        {
            if(motor.getActivated() && motor.getMaintainPositionActive())
            {
                updateMotorMaintainPosition(motor);
            }
        }
        super.loop();
    }
    //whether the intake is state based or continuous
    protected boolean intakeContinuous = false;
    //current state of the intakeServos
    private boolean intakeState = false;

    //power of the intake at full
    private static double intakePower = 0.8;

    //powers of the winch's main and arm motors when raising/lowering
    private static double winchMainRaisePower = 1, winchArmRaisePower = 1, winchMainLowerPower = -1, winchArmLowerPower = -0.7;

    private static double scoopMin = 0.2, scoopMax = 0.8;
    private static double slideMin = 0.5, slideMax = 1;

    private double slidePos = 1;

    //The max and min encoder ticks of the lifter slide
    protected static int liftMax = 26200;
    protected static int liftMin = 8;
    protected int liftTargetPosition = 0;

    protected int armTargetPosition;
    private static int armMinPosition = -20000;
    private static int armMaxPosition = 200;

    private static int armIncrementRatio = 50;
    private static final Pid.PIDConstants ARM_PID_CONSANTS = new Pid.MotorPIDConstants(0.01,0.1,0.7,-280,280);

    protected static final int LIFT_ALIGN_HEIGHT = 20000;
}