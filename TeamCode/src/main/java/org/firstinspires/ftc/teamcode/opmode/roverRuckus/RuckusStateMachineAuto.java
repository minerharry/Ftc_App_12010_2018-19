package org.firstinspires.ftc.teamcode.opmode.roverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.base.StateMachine;
import org.firstinspires.ftc.teamcode.base.StateMachine.*;

import java.util.ArrayList;

@Autonomous(name="Autonomous State Machine Test")
public class RuckusStateMachineAuto extends RuckusRobotHardware {

    StateMachine myMachine = new StateMachine();

    @Override
    public void init() {
        super.init();
        LinearStateTemplate templateA = new StringDurationTemplate(15, "Templated state a");
        LinearStateTemplate templateB = new StringDurationTemplate(15, "Templated state B");
        ArrayList<LinearStateTemplate> templates = new ArrayList<>();
        templates.add(templateA);
        templates.add(templateB);
        State last = new DisplayStringForDuration(7, "End", State.END);
        State next = new StateGroup(templates, last);

        myMachine.startMachine(new DisplayStringForDuration(7.5, "First State", next));

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
            if (Math.abs(currentPosition - myTarget) < myTolerance) {
                return next;
            }
            return this;
        }

        private double myPower;
        private int myTarget;
        private MotorName myMotor;
        private int myTolerance = 10;
    }


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
                    myStates.set(i,currentState.update());
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
