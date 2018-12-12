package org.firstinspires.ftc.teamcode.opmode.roverRuckus;

import org.firstinspires.ftc.teamcode.base.StateMachine;
import org.firstinspires.ftc.teamcode.base.StateMachine.*;

import java.util.ArrayList;

public class RuckusStateMachineAuto extends RuckusRobotHardware {

    public static ArrayList<State> assembleTemplates(ArrayList<StateTemplate> templates,State nextState)
    {
        ArrayList<State> result = new ArrayList<>();
        State next = nextState;
        for(int i = templates.size()-2; i>=0; i--)
        {
            nextState = templates.get(i).makeState(nextState);
            result.add(0,nextState);
        }
        return result;
    }
    private class StateGroup implements State
    {
        private ArrayList<State> myStates;
        public StateGroup(ArrayList<StateTemplate> stateTemplates,State nextState)
        {
            myStates = assembleTemplates(stateTemplates,nextState);
        }
        public StateGroup(ArrayList<State> states)
        {
            myStates = states;
        }
        public void start()
        {
            myStates.get(0).start();
        }
        public State update()
        {
            return myStates.get(0).update();
        }
    }



    // State in the machine to wait for a duration.
    private class WaitForDuration implements StateMachine.State {
        public WaitForDuration(double duration, StateMachine.State next) {
            this.duration = duration;
            this.next = next;
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
        public class WaitTemplate implements StateMachine.StateTemplate
        {
            private double duration;
            public WaitTemplate(double time)
            {
                duration = time;
            }

            @Override
            public StateMachine.State makeState(StateMachine.State initial) {
                return new WaitForDuration(duration,initial);
            }
        }

        private double duration;
        private StateMachine.State next;
        private double startTime;
    }

    // State in the machine to wait for a duration.
    private class DropUntilGyro implements StateMachine.State {
        public DropUntilGyro(double duration, StateMachine.State next) {
            this.duration = duration;
            this.next = next;
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
        private StateMachine.State next;
        private double startTime;
    }

    // State in the machine to wait for a duration.
    private class DropUntilEncoder implements StateMachine.State {
        public DropUntilEncoder(double tickLength, RuckusMotorName watchMotor, float power, StateMachine.State next) {
            this.ticks = tickLength;
            this.next = next;
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
        private StateMachine.State next;
        private double startTicks; //the encoder position at t=0
        private RuckusMotorName motor;
        private float hingePower;

    }



}
