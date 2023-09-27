package org.firstinspires.ftc.teamcode.subsystems.fsm;

/**
 * A state, belonging to an abstract state machine.
 */
public interface State {
    /** The edges of the current state. */
    Edge<?>[] getEdges();

    /**
     * Runs ONCE during state machine initialization.
     * <p>
     * Sets up the robot physically, digitally, and mentally for the trials ahead.
     */
    default void init() {
        /* Do nothin'. */
    }

    /**
     * Runs ONCE when the state is very first reached. Very little initialization should be done
     * here, if any.
     */
    default void start() {
        /* Do nothin'. */
    }

    /**
     * Code that is called once per frame if the state is active. Process input, send output to
     * motors, etc. Do it all here!
     */
    default void loop() {
        /* Do nothin'. */
    }
}
