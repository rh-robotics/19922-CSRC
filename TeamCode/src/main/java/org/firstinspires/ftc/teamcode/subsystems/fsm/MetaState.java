package org.firstinspires.ftc.teamcode.subsystems.fsm;

/** A container to hold a state and it's metadata. */
public class MetaState {
    private final State state;
    private StateState stateState;

    /** Constructs a new MetaState. */
    public MetaState(State state, StateState stateState) {
        this.state = state;
        this.stateState = stateState;
    }

    /** Gets the underlying state. */
    public State getState() {
        return state;
    }

    /** Gets the state of the state. */
    public StateState getStateState() {
        return stateState;
    }

    /** Sets the state of the state. */
    public void setStateState(StateState stateState) {
        this.stateState = stateState;
    }
}
