package org.firstinspires.ftc.teamcode.subsystems.fsm;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

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

    /**
     * How to use the state; initial, transitional, or terminating.
     */
    enum Role {
        /** This is the first state to jump to. */
        INITIAL,

        /** This state is nothing special in the eyes of the controller. */
        TRANSITIONAL,

        /** Once this state is run once, the state machine dies. */
        TERMINATING
    }

    /**
     * Describes some metadata about a state.
     */
    @Retention(RetentionPolicy.RUNTIME)
    @Target(ElementType.TYPE)
    @interface Meta {
        /** The color to use when graphing with GraphViz DOT. */
        String color() default "#000000";

        /** How to use the state. */
        Role role() default Role.TRANSITIONAL;
    }
}
