package org.firstinspires.ftc.teamcode.subsystems.fsm;

/** A callback to decide if a state machine should shift state. */
public interface EdgeCallback {
    /** Ditto.
     * @param state The state to evaluate with.
     * @return Should we transition? */
    boolean valve(State state);
}
