package org.firstinspires.ftc.teamcode.opmode.roverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.base.StateMachine;
import org.firstinspires.ftc.teamcode.base.StateMachine.*;
import org.firstinspires.ftc.teamcode.control.Pid;

import java.util.ArrayList;

@Autonomous(name="Autonomous State Machine Test")
public class RuckusStateMachineAuto extends RuckusRobotHardware {

    StateMachine myMachine = new StateMachine();

    @Override
    public void init() {
        super.init();
        /*LinearStateTemplate templateA = new StringDurationTemplate(15, "Templated state a");
        LinearStateTemplate templateB = new StringDurationTemplate(15, "Templated state B");
        ArrayList<LinearStateTemplate> templates = new ArrayList<>();
        templates.add(templateA);
        templates.add(templateB);
        State last = new DisplayStringForDuration(7, "End", State.END);*/

        RuckusMotorName.MAIN_ARM.activate();
        State next = new RunMotorToEncoderLinear(State.END, 1.0, 5000, RuckusMotorName.MAIN_ARM.getMotorName());

        myMachine.startMachine(next);

    }


    @Override
    public void loop() {
        super.loop();
        myMachine.updateMachine();
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


    /**
     * Uses a custom Pid to maintain a motor at a consistent speed until some target position is reached
     * You will need to tune the Pid to get a set of variables stored in the PIDConstants class.
     * For more information about Pid please
     * @see Pid
     */
    private class RunMotorToEncoderPID extends LinearState {
        private Pid myPid;
        private MotorName myMotor;
        private double myStartTime;
        private double lastUpdate;
        private double lastPosition;
        private int myTarget;
        private double myTargetSpeed;
        private int myTolerance = 10;
        /** @param speed - the desired speed in encoder ticks / second**/
        public RunMotorToEncoderPID(MotorName motorName, State nextState, Pid.PIDConstants constants, int target, double speed)
        {
            super(nextState);
            myPid = new Pid(constants);
            myMotor = motorName;
            setMotorType(myMotor, DcMotor.RunMode.RUN_USING_ENCODER);
            myTarget=target;
            myTargetSpeed = speed;
        }
        public void start()
        {
            myStartTime = time;
            lastUpdate=myStartTime;
            lastPosition = getMotorPosition(myMotor);
        }
        public State update()
        {
            double dt = time - lastUpdate;
            lastUpdate=time;
            double tempPosition = getMotorPosition(myMotor);
            setPower(myMotor,myPid.update(myTargetSpeed,(tempPosition-lastPosition)/dt,dt));
            lastPosition = tempPosition;
            if (Math.abs(lastPosition-myTarget) < myTolerance)
            {
                setPower(myMotor,0);
                return next;
            }
            return this;
        }
    }

    /**
     * Basically just a state version of runToPosition; will use the default encoder to run a motor to a position
     * using a certain power. Not recommended for smooth or specific actions; default Pid is not very
     * well tuned. For accurate and smooth movement (ex: arm w/ weight, driving straight/in a specific way,
     * etc.) use the RunMotorToEncoderPid state.
     */
    private class RunMotorToEncoderDefaultPid extends LinearState
    {
        private MotorName myMotor;
        private int myTarget;
        private double myPower;
        private double myTolerance = 10;
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
        }
        public State update()
        {
            if (Math.abs(getMotorPosition(myMotor)-myTarget) < myTarget)
            {
                setMotorType(myMotor,DcMotor.RunMode.RUN_USING_ENCODER);
                setPower(myMotor,0);
                return next;
            }
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
                    currentState.update();
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
                        currentState.update();
                        if (tempState != currentState)
                            stateChanged.set(i, true);
                        else
                            anyUnchanged = true;
                        myStates.set(i, currentState);
                    }
                }
            }
            if (anyActive)
            {
                return this;
            }
            if (!anyUnchanged)
            {
                stateChanged.clear();
                for(int i = 0; i < myStates.size(); i++)
                {
                    stateChanged.add(false);
                    myStates.get(i).start();
                }
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
