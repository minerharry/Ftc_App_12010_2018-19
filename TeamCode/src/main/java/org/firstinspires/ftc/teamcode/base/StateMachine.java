package org.firstinspires.ftc.teamcode.base;

/**
 * Created by Colin_Zhu on 2/10/2018.
 */

/**
 * State machine manager.
 * Simplifies the development of finite state machines.
 * Handles and advances between different states.
 */


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/** About the State machine:
 * A State (defined here) is a customized method wrapped in a class.
 * After set intervals(?), the state machine will call the update() function, which will return the next state
 * Often there is a state that is a WaitFor or WaitTime
 * Statemachine tutorial/support: https://pmtischler-ftc-app.readthedocs.io/en/latest/tutorials/state_machine.html
 */
public class StateMachine {

    /**
     * A state in the state machine.
     */
    public static interface State {
        /**
         * Called when the state first becomes the active state.
         */
        public void start();

        /**
         * Called on each update.
         * @return The next state to run.
         */
        public State update();

        }

    public interface StateTemplate
    {
        State makeState(State initial);
    }
    /**
     * Creates the state machine with the initial state.
     * @param initial The initial state.
     */
    public void startMachine(State initial){
        state = initial;
        state.start();
    }


    /**
     * Performs an update on the state machine.
     */
    public void updateMachine() {
        if (state == null) {
            return;
        }

        State next = state.update();
        if (next != null && state != next) {
            next.start();
        }
        state = next;
    }


    // Gets the current state.
    public State currentState() {
        return state;
    }

    // The current state.
    private State state;

}
