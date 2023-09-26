package org.firstinspires.ftc.teamcode.subsystems.fsm;

import java.util.Arrays;
import java.util.HashMap;

/**
 * Represents a finite state machine.
 */
public class StateMachine {
    /* The flat list of states. The key is the state, the value is if it's been reached. */
    private final HashMap<State, Boolean> states_reached = new HashMap<>();

    /* The current state being executed. */
    private State current = null;

    /**
     * Add a state to the state machine.
     *
     * @param state The state to add.
     * @return The state machine.
     */
    public StateMachine addState(State state) {
        current = current == null ? state : current;
        states_reached.put(state, false);
        state.init();
        return this;
    }

    /**
     * Remove a state from the state machine.
     *
     * @param state The state to remove.
     * @return The state machine.
     */
    public StateMachine deleteState(State state) {
        states_reached.remove(state);
        return this;
    }

    /**
     * Called on every frame of the robot, it ticks and manages the state machine.
     */
    public void loop() {
        /* Sanity. */
        if (states_reached.size() == 0) {
            throw new RuntimeException("No states in state machine.");
        }

        /* Run start() once we reach the state. */
        if (Boolean.FALSE.equals(states_reached.get(current))) {
            current.start();
            states_reached.put(current, true);
        }

        /* Run the current state's looping function. */
        current.loop();

        /*
         * Figure out which, if any, state the current state wants to pass off execution to.
         * Filter out all the closed valves, get all the states that the true valves point to,
         * and set that to the current.
         */
        current = Arrays.stream(current.edges).filter(e -> e.getCallback().valve(current)).map((e) -> {
            // TODO: Handle multiple true valves instead of blindly accepting the first.
            return states_reached.keySet().stream().filter((s) -> {
                //noinspection CodeBlock2Expr
                return s.getClass().isInstance(e.getTo());
            }).findFirst().orElseThrow(() -> new RuntimeException("Valve function referenced non-existent state '" + e.getTo() + "'."));
        }).findFirst().orElseThrow(() -> new RuntimeException("State '" + current.getClass().getSimpleName() + "' has no children."));
    }
}
