package org.firstinspires.ftc.teamcode.base;

/**
 * Created by Colin_Zhu on 2/10/2018.
 */

/**
 * State machine manager.
 * Simplifies the development of finite state machines.
 * Handles and advances between different states.
 */


/** About the State machine:
 * A State (defined here) is a customized method wrapped in a class.
 * After set intervals(?), the state machine will call the update() function, which will return the next state
 * Often there is a state that is a WaitFor or WaitTime
 *
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

    /**
     * Creates the state machine with the initial state.
     * @param initial The initial state.
     */
    public StateMachine(State initial) {
        state = initial;
        state.start();
    }


    /**
     * Performs an update on the state machine.
     */
    public void update() {
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
