package org.firstinspires.ftc.teamcode.subsystems.fsm;

/** A container to hold a state and it's metadata. */
public class MetaState {
    private final State state;
    private StateState stateState;

    /** Constructs a new MetaState.
     * @param state The underlying state.
     * @param stateState The state of the state.
     */
    public MetaState(State state, StateState stateState) {
        this.state = state;
        this.stateState = stateState;
    }

    /** Gets the underlying state.
     * @return Ditto. */
    public State getState() {
        return state;
    }

    /** Gets the state of the state.
     * @return Ditto. */
    public StateState getStateState() {
        return stateState;
    }

    /** Sets the state of the state.
     * @param stateState Ditto. */
    public void setStateState(StateState stateState) {
        this.stateState = stateState;
    }
}
