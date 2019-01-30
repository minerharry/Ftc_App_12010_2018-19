package org.firstinspires.ftc.teamcode.opmode.roverRuckus;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.base.StateMachine;
import org.firstinspires.ftc.teamcode.base.StateMachine.*;
import org.firstinspires.ftc.teamcode.control.Pid;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

/**
 * A class that runs a state machine for the Rover Ruckus Challenge. State extending classes are nested here,
 * specifically all of the ones needed for this challenge in particular. ATM, all of the states
 * are here; ASAP: Create general class w/ states for: Motors, gyro, color sensors, etc.
 * Order of events in a state:
 *  --Constructor - happens before the super.init() in the main constructor - activate the objects used in a state when passed to ta
 *     to that state's constructor, but do not reference those objects in that state's constructor
 *  --start() - happens at the very first loop() function; have all access to hardware from the RobotHardware class
 *  --update() - happens every frame during loop ()
 */
@TeleOp(name="Autonomous Test debugging")
public class RuckusStateMachineAuto extends RuckusRobotHardware {
    protected String RobotRotationGyroName = "Z angle Robot Rotation Gyro";
    StateMachine myMachine = new StateMachine();
    boolean started = false;
    State startingState;
    Map<String,BackgroundState> backgroundStates;
    @Override
    public void init() {
        backgroundStates = new HashMap<>();
        for (RuckusMotorName motorName : tankMotors)
        {
            motorName.activate();
        }
        startingState = new ScanUntilGoldAligned(0,new DisplayStringForDuration(45, "Left State", State.END), new DisplayStringForDuration(45,"Middle State",State.END), new DisplayStringForDuration(45, "Right State", State.END));

        super.init();
        /*LinearStateTemplate templateA = new StringDurationTemplate(15, "Templated state a");
        LinearStateTemplate templateB = new StringDurationTemplate(15, "Templated state B");
        ArrayList<LinearStateTemplate> templates = new ArrayList<>();
        templates.add(templateA);
        templates.add(templateB);
        State last = new DisplayStringForDuration(7, "End", State.END);*/
        for (RuckusMotorName motorName : RuckusRobotHardware.tankMotors)
        {
            setMotorType(motorName.getMotorName(),DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    @Override
    public void loop() {

        if (!started)
        {
            for (String key : backgroundStates.keySet())
            {
                backgroundStates.get(key).start();
            }
            myMachine.startMachine(startingState);
            started = true;

        }
        super.loop();
        if  (!gamepad1.a) {
            myMachine.updateMachine();
            for (String key : backgroundStates.keySet())
            {
                backgroundStates.put(key,backgroundStates.get(key).update());
            }
        }
    }

    @Override
    public void stop() {
        for (String key : backgroundStates.keySet())
        {
            backgroundStates.get(key).stop();
        }
    }

    public static ArrayList<LinearState> assembleTemplates(ArrayList<LinearStateTemplate> templates, State nextState) {
        ArrayList<LinearState> result = new ArrayList<>();
        LinearState newState;
        for (int i = templates.size() - 1; i >= 0; i--) {
            newState = templates.get(i).makeState(nextState);
            result.add(0, newState);
            nextState = newState;
        }
        return result;
    }

    public String ActivateRobotRotationGyro()
    {
        telemetry.addData("Gyro status: ", "Background states");
        if (backgroundStates.get(RobotRotationGyroName) == null)
        {
            backgroundStates.put(RobotRotationGyroName,new UpdateGyroAngle(RuckusGyroName.HUB_2_IMU,AxesOrder.ZYX));
        }
        return RobotRotationGyroName;
    }
    private class UpdateGyroAngle extends BackgroundState {
        private AxesOrder myOrder;
        private BNO055IMU imu;
        private RuckusGyroName gyroName;
        private double lastAngle;
        private double myAngle;
        boolean stopped = false;
        public UpdateGyroAngle(RuckusGyroName imuName, AxesOrder order)
        {
            gyroName = imuName;
            gyroName.activate();
            myOrder = order;
        }
        public void start()
        {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.mode                = BNO055IMU.SensorMode.IMU;
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled      = true;
            gyroName.setParameters(parameters);
        }
        public BackgroundState update()
        {
            imu = getGyro(gyroName.getGyroName());
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC,myOrder,AngleUnit.DEGREES);
            float angle  = angles.firstAngle;
            angle = ( angle < -180 ? angle + 360 : (angle > 180 ? angle - 360 : angle));
            double deltaAngle = lastAngle - angle;
            deltaAngle = ( deltaAngle < -180 ? deltaAngle + 360 : (deltaAngle > 180 ? deltaAngle - 360 : deltaAngle));
            myAngle += deltaAngle;
            lastAngle = angles.firstAngle;
            if (!stopped) {
                return this;
            }
            return BackgroundState.END;
        }
        public void resetAngle()
        {
            myAngle = 0;
        }
        public void resetAngle(double angle)
        {
            myAngle = angle;
        }
        public double getAngle()
        {
            return myAngle;
        }
    }

    /**
     * A class for testing the dogeCV and gyro implementation
     * does a 360, constantly displaying the x coordinate of its current best gold match
     * and its current angle
     */
    private class SpinForGold extends LinearState {
        private RotateRobotByGyro rotator;
        private GoldAlignDetector detector;
        public SpinForGold(State nextState)
        {
            super(nextState);
            rotator = new RotateRobotByGyro(360, 0.2, State.END, true);
            detector = new GoldAlignDetector();
            detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
            detector.useDefaults();

            // Optional Tuning
            detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
            detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
            detector.downscale = 0.4; // How much to downscale the input frames

            detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
            //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
            detector.maxAreaScorer.weight = 0.005;

            detector.ratioScorer.weight = 5;
            detector.ratioScorer.perfectRatio = 1.0;
            detector.verticalMax = 0.9;
            detector.enable();
        }
        @Override
        public void start()
        {
            rotator.start();
        }
        @Override
        public State update() {
            telemetry.addData("Gold X Position", detector.getXPosition());
            telemetry.addData("Current Angle", rotator.getAngle());
            if (detector.isFound() && (detector.getXPosition() < 400 && detector.getXPosition() > 300)) {
                telemetry.addData("Gold Found","True");
                return next;
            }
            State tempState = rotator.update();
            if (tempState == State.END)
            {
                return next;
            }
            return this;
        }
    }
    /**
     * Uses either RotateByEncoder or RotateByGyro to turn left and right and find the gold mineral using DogeCV's
     * GoldAlignDetector
     * Will return one of three statepaths provided in constructor, and will use the general FOV (in degrees in front of the robot)
     * to judge how far it should turn
     * should start facing the middle of the three minerals at a set distance away from the line (which is a constant
     * START_DISTANCE in inches to the center of rotation of the robot, and uses trig about the distance
     * between each mineral (14.5 in)
     */
    private class ScanUntilGoldAligned implements State {
        private State left;
        private State middle;
        private State right;
        private GoldAlignDetector detector;
        private RotateRobotByGyro rotator;
        private final double MINERAL_SPACING = 14.5; //space between each mineral in inches
        private final double DISTANCE_TO_LINE = 24; //average distance from robot to center mineral in inches after landing
        double turnExtent = 45.0;
        int rotatePhase;
        double firstAngle;
        boolean[] detection;

        public ScanUntilGoldAligned(double fov, State leftState, State middleState, State rightState)
        {
            left = leftState;
            middle = middleState;
            right = rightState;

            detector = new GoldAlignDetector();
            detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
            detector.useDefaults();

            // Optional Tuning
            detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
            detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
            detector.downscale = 0.4; // How much to downscale the input frames

            detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
            //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
            detector.maxAreaScorer.weight = 0.005;

            detector.ratioScorer.weight = 5;
            detector.ratioScorer.perfectRatio = 1.0;
            detector.verticalMax = 0.9;
            detector.enable();
            turnExtent = Math.atan2(MINERAL_SPACING,DISTANCE_TO_LINE)*180/Math.PI + 15;
            telemetry.addData("Turn Extent",turnExtent);
            RotateRobotByGyro rotator3 = new RotateRobotByGyro(-turnExtent, 0.6, State.END, true);
            RotateRobotByGyro rotator2 = new RotateRobotByGyro(turnExtent*2, 0.6, rotator3, true);
            rotator = new RotateRobotByGyro(-turnExtent, 0.8, rotator2,true);

        }
        public void start()
        {
            rotator.start();
            firstAngle = ((UpdateGyroAngle) backgroundStates.get(RobotRotationGyroName)).getAngle();
        }
        public State update()
        {
            State tempState = rotator.update();
            if(tempState != rotator)
            {
                rotatePhase++;
                switch (rotatePhase)
                {
                    case 1:
                    {
                        telemetry.addData("Turn Status","Left Completed");
                        break;
                    }
                    case 2:
                    {
                        telemetry.addData("Turn Status", "Right completed");
                    }
                    case 3:
                    {
                        telemetry.addData("Turn Status", "Middle Completed");
                    }

                }
            }
            if (tempState == State.END)
            {
                telemetry.addData("Gold", "Not Found");
                return middle;
            }
            rotator = (RotateRobotByGyro) tempState;
            if (detector.getAligned())
            {
                double angle = rotator.getAngle();
                telemetry.addData("Gold", "Found");
                if(angle< -15)
                {
                    rotator.stop();
                    return left;
                }
                else if(angle >= -15 && angle <= 15)
                {
                    rotator.stop();
                    return middle;
                }
                else
                {
                    rotator.stop();
                    return right;
                }

            }
            telemetry.addData("Gold","Still Searching");
            return this;
        }

    }


    /**
     * Uses a custom Pid to maintain a motor at a consistent speed until some target position is reached
     * You will need to tune the Pid to get a set of variables stored in the PIDConstants class.
     * For more information about Pid please
     * @see Pid
     */
    private class RunMotorToEncoderPID extends LinearState {
        private Pid myPid;
        private MotorName myMotor; //the motor
        private double myStartTime; //the time the state started
        private double lastUpdate; //the last time of the update
        private double lastPosition; //the position of the motor at the last interval
        private int myTarget; //the target position
        private double myTargetSpeed;
        private int myTolerance = 10;
        /** @param speed - the desired speed in encoder ticks / second**/
        public RunMotorToEncoderPID(MotorName motorName, State nextState, Pid.PIDConstants constants, int target, double speed)
        {
            super(nextState);
            myPid = new Pid(constants);
            myMotor = motorName;
            myTarget=target;
            myTargetSpeed = speed;
        }
        public void start()
        {
            setMotorType(myMotor, DcMotor.RunMode.RUN_USING_ENCODER);
            myStartTime = time;
            lastUpdate=myStartTime;
            lastPosition = getMotorPosition(myMotor);
        }
        public State update()
        {
            double dt = time - lastUpdate;
            lastUpdate=time;
            double tempPosition = getMotorPosition(myMotor);
            setPower(myMotor,myPid.update(Math.signum(myTarget-tempPosition)*myTargetSpeed,(tempPosition-lastPosition)/dt,dt));
            lastPosition = tempPosition;
            if (Math.abs(lastPosition-myTarget) < myTolerance)
            {
                setPower(myMotor,0);
                return next;
            }
            return this;
        }
    }
    protected enum PidType{
        TYPE_DEFAULT,
        TYPE_CUSTOM,
        TYPE_LINEAR
    }

    /**
     * State that will rotate the robot using the revhub built in IMU.
     * Will either increment by an amount or set it to an angle, using the global angle since the last reset;
     * Uses a background state to get the Z angle
     */
    private class RotateRobotByGyro extends LinearState{
        private double myTolerance = 6; //gyro tolerance in degrees
        private double myTarget;
        private double myPower;
        private String gyroBackgroundName;
        private UpdateGyroAngle gyro;

        public double getAngle() {
            return gyro.getAngle();
        }
        public void stop() {
            for(RuckusMotorName motor : tankMotors)
            {
                setMotorTargetPosition(motor.getMotorName(),getMotorTargetPosition(motor.getMotorName()));
                setPower(motor.getMotorName(),0);
            }
        }

        private boolean incrementTarget;
        public RotateRobotByGyro(double target, double power, State nextState, boolean increment)
        {
            super(nextState);
            myPower = Math.abs(power);
            myTarget = target;
            incrementTarget = increment;
            gyroBackgroundName = ActivateRobotRotationGyro();
            gyro = ((UpdateGyroAngle) backgroundStates.get(gyroBackgroundName));
        }

        @Override
        public void start() {
            for (RuckusMotorName name : tankMotors)
            {
                setMotorType(name.getMotorName(), DcMotor.RunMode.RUN_TO_POSITION);
                setPower(name.getMotorName(), myPower);
                setMotorTargetPosition(name.getMotorName(),getMotorPosition(name.getMotorName()));
            }
            double angle = getAngle();
            if (!incrementTarget) {
                angle = (angle + 360) % 360;
                myTarget = (myTarget + 720) % 360; //ensuring both angles are within 0-360
                double d1 = angle - myTarget; // angle between robot and target within the 0-360 domain
                myTarget = (Math.abs(d1) > 180 ? (d1 > 0 ? myTarget + 360 : myTarget - 360) : myTarget);
            }
            else
            {
                myTarget = angle + myTarget;
            }
            gyro.resetAngle(angle);
            /* if d1 is > 180, that means that the shortest angle is in the opposite direction
            Which depends on whether angle > than mytarget (aka d1 > 0), and will increment mytarget by one 360 rotation so that the direction is then shorter


            */
        }

        @Override
        public State update() {

            telemetry.addData("current Position: ", getAngle());
            telemetry.addData("Target Position:", myTarget);


            if (Math.abs(getAngle() - myTarget) < myTolerance)
            {
                for(RuckusMotorName motor : tankMotors)
                {
                    setMotorTargetPosition(motor.getMotorName(),getMotorTargetPosition(motor.getMotorName()));
                    setPower(motor.getMotorName(),0);
                }
                return next;
            }
            if (getAngle() - myTarget < 0)
            {
                for (RuckusMotorName motor : tankMotors)
                {
                    setMotorTargetPosition(motor.getMotorName(),getMotorTargetPosition(motor.getMotorName())-20000);
                }
            }
            if (getAngle() - myTarget > 0)
            {
                for (RuckusMotorName motor : tankMotors)
                {
                    setMotorTargetPosition(motor.getMotorName(),getMotorTargetPosition(motor.getMotorName())+20000);
                }
            }
            return this;
        }
    }

    /**
     * A simple class that uses a generated parallelstategroup to run the four drive motors in the same direction by some given number of encoder ticks
     * Positive ticks rotates counterclockwise, negative rotates clockwise
     */
    private class RotateRobotByEncoders extends LinearState{
        private Pid.PIDConstants DEFAULT_TURN_PID;
        private ParallelStateGroup motorGroup;
        public RotateRobotByEncoders(int target, PidType type, double power, State nextState)
        {
            super(nextState);
            if (type == PidType.TYPE_DEFAULT) {

                State motorBL = new RunMotorToEncoderDefaultPid(RuckusMotorName.DRIVE_BACK_LEFT.getMotorName(), getMotorTargetPosition(RuckusMotorName.DRIVE_BACK_LEFT.getMotorName()) + target, power, State.END);
                State motorBR = new RunMotorToEncoderDefaultPid(RuckusMotorName.DRIVE_BACK_RIGHT.getMotorName(), getMotorTargetPosition(RuckusMotorName.DRIVE_BACK_RIGHT.getMotorName()) + target, power, State.END);
                State motorFL = new RunMotorToEncoderDefaultPid(RuckusMotorName.DRIVE_FRONT_LEFT.getMotorName(), getMotorTargetPosition(RuckusMotorName.DRIVE_FRONT_LEFT.getMotorName()) + target, power, State.END);
                State motorFR = new RunMotorToEncoderDefaultPid(RuckusMotorName.DRIVE_FRONT_RIGHT.getMotorName(), getMotorTargetPosition(RuckusMotorName.DRIVE_FRONT_RIGHT.getMotorName()) + target, power, State.END);
                ArrayList<State> states = new ArrayList<>();
                states.add(motorBL);
                states.add(motorBR);
                states.add(motorFL);
                states.add(motorFR);
                motorGroup = new ParallelStateGroup(next,states);
            }

        }

        @Override
        public void start() {

        }

        @Override
        public State update() {
            return motorGroup;
        }
    }

    /**
     * Basically just a state version of setMotorTargetPosition; will use the default encoder to run a motor to a position
     * using a certain power.
     */
    private class RunMotorToEncoderDefaultPid extends LinearState
    {
        private MotorName myMotor;
        private int myTarget;
        private double myPower;
        private double myTolerance = 10;
        private double timeWithinTolerance;
        private boolean lastWithinTolerance;
        public RunMotorToEncoderDefaultPid(MotorName motorName, int target, double power, State nextState)
        {
            super(nextState);
            myTarget = target;
            myPower = power;
            myMotor = motorName;
        }
        public void start()
        {
            setMotorType(myMotor, DcMotor.RunMode.RUN_TO_POSITION);
            setPower(myMotor,myPower);
            setMotorTargetPosition(myMotor,myTarget);
        }
        public State update()
        {
            if (Math.abs(getMotorPosition(myMotor)-myTarget) < myTolerance)
            {
                if (!lastWithinTolerance)
                {
                    timeWithinTolerance = time;
                }
                lastWithinTolerance = true;
                if (timeWithinTolerance - time > 0.75) {
                    setMotorType(myMotor, DcMotor.RunMode.RUN_USING_ENCODER);
                    setPower(myMotor, 0);
                    return next;
                }
            }
            telemetry.addData("Motor Running: " + myMotor.getName(), "Current Position: " + getMotorPosition(myMotor));
            return this;
        }

    }
    private class RunMotorToEncoderLinear extends LinearState {
        public RunMotorToEncoderLinear(State nextState, double power, int position, MotorName motorName) {
            super(nextState);
            myPower = power;
            myTarget = position;
            myMotor = motorName;
            setMotorType(motorName, DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public RunMotorToEncoderLinear(State nextState, double power, int position, MotorName motorName, int tolerance) {
            super(nextState);
            myPower = power;
            myTarget = position;
            myMotor = motorName;
            myTolerance = tolerance;
        }

        @Override
        public void start() {
            setMotorType(myMotor, DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public State update() {
            int currentPosition = getMotorPosition(myMotor);
            setPower(myMotor, myPower * (myTarget - currentPosition < 0 ? -1 : 1));
            telemetry.addData("Motor Position:", getMotorPosition(myMotor));
            telemetry.addData("Target Position:", myTarget);
            if (Math.abs(currentPosition - myTarget) < myTolerance) {
                setPower(myMotor,0);
                return next;
            }
            return this;
        }

        private double myPower;
        private int myTarget;
        private MotorName myMotor;
        private int myTolerance = 10;
    }

    /**A group of states that will run and update all states at the same time, and will then return the 'nextState'
    * when the last remaining state is finished. Note: Will fully evaluate all component states and their
    * trees. Only one next state will be called after the group is finished, and that is the argument nextState
    * Make sure that each passed state either has a next of State.END or leads to the chain that you want it to.
    * Group will not call all following states at the same time; use SyncedParallelStateGroup instead**/
    private class ParallelStateGroup extends LinearState
    {
        private ArrayList<State> myStates;
        public ParallelStateGroup(ArrayList<LinearStateTemplate> stateTemplates, State nextState)
        {
            super(nextState);
            myStates = new ArrayList<>();
            for(LinearStateTemplate template : stateTemplates)
            {
                myStates.add(template.makeState(State.END));
            }
        }
        public ParallelStateGroup(State nextState, ArrayList<State> states)
        {
            super(nextState);
            myStates = states;
        }
        public void start()
        {
            for(State state : myStates)
            {
                state.start();
            }
        }
        public State update()
        {
            boolean anyActive = false;
            for(int i = 0; i < myStates.size(); i++)
            {
                State currentState = myStates.get(i);
                if (currentState != State.END)
                {
                    anyActive = true;
                    State tempState = currentState;
                    currentState = currentState.update();
                    if (tempState != currentState)
                        currentState.start();
                    myStates.set(i,currentState);
                }
            }
            if (anyActive)
            {
                return this;
            }
            return next;
        }
    }
    private class ParallelGroupTemplate implements LinearStateTemplate
    {
        private ArrayList<State> myStates;
        public ParallelGroupTemplate(ArrayList<State> states)
        {
            myStates = states;
        }

        @Override
        public LinearState makeState(State nextState) {
            return new ParallelStateGroup(nextState,myStates);
        }
    }

    /**
     * acts similarly to ParallelStateGroup, except for all following states will wait to start them
     */
    private class SyncedParallelStateGroup extends LinearState
    {
        private ArrayList<State> myStates;
        private ArrayList<Boolean> stateChanged;
        public SyncedParallelStateGroup(ArrayList<LinearStateTemplate> stateTemplates, State nextState)
        {
            super(nextState);
            myStates = new ArrayList<>();
            stateChanged = new ArrayList<>();
            for(LinearStateTemplate template : stateTemplates)
            {
                myStates.add(template.makeState(State.END));
                stateChanged.add(false);
            }

        }
        public SyncedParallelStateGroup(State nextState, ArrayList<State> states)
        {
            super(nextState);
            myStates = states;
            stateChanged = new ArrayList<>();
            for (State state : myStates)
            {
                stateChanged.add(false);
            }
        }
        public void start()
        {
            for(State state : myStates)
            {
                state.start();
            }
        }
        public State update()
        {
            boolean anyActive = false;
            boolean anyUnchanged = false;
            for(int i = 0; i < myStates.size(); i++)
            {
                State currentState = myStates.get(i);
                if (currentState != State.END)
                {
                    anyActive = true;
                    if (!stateChanged.get(i)) {
                        State tempState = currentState;
                        currentState = currentState.update();
                        if (tempState != currentState)
                            stateChanged.set(i, true);
                        else
                            anyUnchanged = true;
                        myStates.set(i, currentState);
                    }
                }
            }
            if (!anyUnchanged)
            {
                telemetry.addData("Unchanged Status", "None unchanged");
                stateChanged.clear();
                for(int i = 0; i < myStates.size(); i++)
                {
                    stateChanged.add(false);
                    myStates.get(i).start();
                }
            }
            if (anyActive)
            {
                return this;
            }

            return next;
        }
    }
    private class SyncedParallelGroupTemplate implements LinearStateTemplate
    {
        private ArrayList<State> myStates;
        public SyncedParallelGroupTemplate(ArrayList<State> states)
        {
            myStates = states;
        }

        @Override
        public LinearState makeState(State nextState) {
            return new SyncedParallelStateGroup(nextState,myStates);
        }
    }
    private class StateGroup extends LinearState
    {
        private ArrayList<LinearState> myStates;
        private State currentState;
        public StateGroup(ArrayList<LinearStateTemplate> stateTemplates, State nextState)
        {
            super(nextState);
            myStates = assembleTemplates(stateTemplates,nextState);
        }
        public StateGroup(ArrayList<LinearState> states)
        {
            super(states.get(states.size()-1).next);
            myStates = states;
        }
        public void start()
        {
            currentState = myStates.get(0);
            currentState.start();
        }
        public State update()
        {
            currentState = currentState.update();
            if (currentState != State.END)
            {
                return currentState;
            }
            return this;
        }

    }

    public class GroupTemplate implements LinearStateTemplate {
        public GroupTemplate(ArrayList<LinearStateTemplate> templates)
        {
            myTemplates = templates;
        }
        public LinearState makeState(State nextState)
        {
            return new StateGroup(myTemplates,nextState);
        }
        protected ArrayList<LinearStateTemplate> myTemplates;
    }


    private class DisplayStringForDuration extends LinearState {
        public DisplayStringForDuration(double duration, String text, State nextState) {
            super(nextState);
            myDuration = duration;
            myText = text;
        }

        @Override public void start() {startTime = time;}

        @Override
        public State update()
        {
            double elapsed = time - startTime;
            telemetry.addData("Elapsed", elapsed);
            telemetry.addData("Duration", myDuration);
            telemetry.addData("Text Output:",myText);
            if (elapsed > myDuration) {
                telemetry.addData("Text Output:","State Ended");
                return next;
            }
            return this;
        }
        private String myText;
        private double myDuration;
        private double startTime;
    }

    public class StringDurationTemplate implements LinearStateTemplate{
        public StringDurationTemplate(double duration, String text)
        {
            templateDuration = duration;
            templateString = text;
        }
        @Override
        public LinearState makeState(State nextState) {
            return new DisplayStringForDuration(templateDuration,templateString,nextState);
        }
        private String templateString;
        private double templateDuration;
    }
    // State in the machine to wait for a duration.
    private class WaitForDuration extends LinearState {
        public WaitForDuration(double duration, StateMachine.State nextState) {
            super(nextState);
            this.duration = duration;
        }

        @Override
        public void start() {
            startTime = time;
        }

        @Override
        public StateMachine.State update() {
            double elapsed = time - startTime;
            telemetry.addData("Elapsed", elapsed);
            telemetry.addData("Duration", duration);
            if (elapsed > duration) {
                return next;
            }
            return this;
        }
        public class WaitTemplate implements LinearStateTemplate
        {
            private double myDuration;
            public WaitTemplate(double time)
            {
                myDuration = time;
            }

            @Override
            public LinearState makeState(StateMachine.State initial) {
                return new WaitForDuration(myDuration,initial);
            }
        }

        private double duration;
        private double startTime;
    }

    // State in the machine to wait for a duration.
    private class DropUntilGyro extends LinearState {
        public DropUntilGyro(double duration, StateMachine.State nextState) {
            super(nextState);
            this.duration = duration;
        }

        @Override
        public void start() {
            startTime = time;
        }

        @Override
        public StateMachine.State update() {
            double elapsed = time - startTime;
            telemetry.addData("Elapsed", elapsed);
            telemetry.addData("Duration", duration);
            if (elapsed > duration) {
                return next;
            }
            return this;
        }
        //public class DropGyroTemplate TODO: finish this when gyro figured out

        private double duration;
        private double startTime;
    }

    // State in the machine to wait for a duration.
    private class DropUntilEncoder extends LinearState {
        public DropUntilEncoder(double tickLength, RuckusMotorName watchMotor, float power, StateMachine.State nextState) {
            super(nextState);
            this.ticks = tickLength;
            this.motor = watchMotor;
            this.hingePower = power;
        }

        @Override
        public void start() {
            startTicks = getMotorPosition(motor.getMotorName());
            setHingePower(hingePower);
        }

        @Override
        public StateMachine.State update() {
            double ticksGone = getMotorPosition(motor.getMotorName()) - startTicks;
            telemetry.addData("Ticks Moved", ticksGone);
            telemetry.addData("Total Ticks:", ticks);
            if (ticksGone > ticks) {
                setHingePower(0);
                return next;
            }
            return this;
        }

        private double ticks; //the number of encoder ticks to move until finished
        private double startTicks; //the encoder position at t=0
        private RuckusMotorName motor;
        private float hingePower;

    }



}