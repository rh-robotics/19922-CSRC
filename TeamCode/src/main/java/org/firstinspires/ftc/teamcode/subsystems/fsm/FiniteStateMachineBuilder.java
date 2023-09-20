package org.firstinspires.ftc.teamcode.subsystems.fsm;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/** Define a state machine before constructing it fully and locking it into place. */
public class FiniteStateMachineBuilder {
    private ArrayList<State> states;

    /** Bail if the state machine isn't deterministic. The default is true because:
     *      A) it's good practice; and
     *      B) it's generally more performant.
     */
    private boolean enforceDeterministicity = true;

    /** The transition function in a DFSM is deterministic, meaning it maps a state and an input
     *  symbol to a unique next state. In an NFSM, it's non-deterministic, allowing multiple
     *  possible next states for a given state and input symbol.
     *  <p>
     *  "For any non-deterministic finite-state machine, an equivalent deterministic one can be
     *  constructed." -- Wikipedia
     * @param enforceDeterministicity Whether to enforce deterministicity.
     */
    public void enforceDeterministicity(boolean enforceDeterministicity) {
        this.enforceDeterministicity = enforceDeterministicity;
    }

    /** Add a state to the state machine. The first state added is the initial state.
     * @param state The state to add.
     * @return Returns the updated builder. */
    public FiniteStateMachineBuilder addState(State state) {
        states.add(state);
        return this;
    }

    /** Removes a state from the state machine.
     * @param state The state to remove.
     * @return Returns the updated builder. */
    public FiniteStateMachineBuilder deleteState(State state) {
        states.remove(state);
        return this;
    }

    /** Gets an unmodifiable list of added states.
     * @return Returns all the associated states, in an unmodifiable form.
     */
    public List<State> getStates() {
        return Collections.unmodifiableList(states);
    }

    /** Gets if deterministicity is enforced.
     * @return Is deterministicity enforced?
     */
    public boolean isEnforceDeterministicity() {
        return enforceDeterministicity;
    }
}
