package org.firstinspires.ftc.teamcode.subsystems.fsm;

/**
 * How to use the state; initial, transitional, or terminating.
 */
public enum StateUsage {
    /** This is the first state to jump to. */
    INITIAL,

    /** This state is nothing special in the eyes of the controller. */
    TRANSITIONAL,

    /** Once this state is run once, the state machine dies. */
    TERMINATING
}
