package org.firstinspires.ftc.teamcode.subsystems.fsm;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Objects;
import org.firstinspires.ftc.teamcode.subsystems.fsm.State.Role;
import org.firstinspires.ftc.teamcode.subsystems.fsm.State.Meta;

/**
 * Represents a finite state machine.
 */
public class StateMachine {
    /**
     * A runtime exception representing an insane state machine configuration encountered at runtime.
     */
    public static class InsaneStateMachineException extends RuntimeException {
        public InsaneStateMachineException(String reason) {
            super("Encountered an insane state machine: " + reason + ".");
        }
    }

    /**
     * A runtime exception representing a malformed state encountered at state machine construction.
     */
    public static class MalformedStateException extends RuntimeException {
        public MalformedStateException(Class<?> offender, String reason) {
            super("State '" + offender.getSimpleName() + "' is malformed: " + reason + ".");
        }
    }

    /**
     * A runtime exception representing an error that occurred during state machine construction, not
     * runtime.
     */
    public static class StateMachineConstructionException extends RuntimeException {
        public StateMachineConstructionException(Class<?> offender, String reason) {
            super("State machine construction is invalid, caused by state '" +
                    offender.getSimpleName() + "': " + reason + ".");
        }
    }

    /**
     * A runtime exception representing an unknown state.
     * <br/>
     * TODO: Figure out if this can be collapsed and covered under InsaneStateMachinrException.
     */
    public static class UnknownStateException extends RuntimeException {
        public UnknownStateException(Class<?> offender) {
            super("Valve function referenced unknown state, '" + offender.getSimpleName() + "'.");
        }
    }

    /* The flat list of states. The key is the state, the value is if it's been reached. */
    private final HashMap<State, Boolean> statesReached = new HashMap<>();

    /* Unique state registry. */
    private final HashMap<Role, State> uniqueStateRegistry = new HashMap<>();

    /* The current state. */
    private State currentState;

    /* Are we done? */
    private boolean terminated = false;

    /* TODO: There's a lot work work here done on the addage of every state. This only needs
     *   to happen at the end. A StateMachineBuilder and a StateMachine, like originally
     *   architected, would solve this. */
    /**
     * Add a state to the state machine.
     *
     * @param state The state to add.
     * @return The state machine.
     */
    public StateMachine addState(State state) {
        Meta metadata;

        /* Throw an exception if a state of the type passed into this method is already present,
         * so we don't fail and overwrite silently. */
        if (statesReached.containsKey(state)) {
            throw new StateMachineConstructionException(state.getClass(), "state of ditto type " +
                    "already included in state machine");
        }

        /* Make sure we have the annotation needed, and load them. */
        if (state.getClass().isAnnotationPresent(Meta.class)) {
            metadata = state.getClass().getAnnotation(Meta.class);
            assert metadata != null;
        } else {
            throw new MalformedStateException(state.getClass(), "isn't annotated with StateMeta");
        }

        /* Ensure there's only ever one INITIAL and TERMINATING state. */
        Arrays.stream(new Role[] {Role.INITIAL, Role.TERMINATING}).forEach(usage -> {
            if (metadata.role().equals(usage)) {
                if (uniqueStateRegistry.containsKey(usage)) {
                    throw new StateMachineConstructionException(state.getClass(), "another state " +
                            "marked as " + usage.name() + ", '" +
                            Objects.requireNonNull(uniqueStateRegistry.get(usage)).getClass().getSimpleName() +
                            "', is already included in state machine."
                    );
                }

                uniqueStateRegistry.put(usage, state);
            }
        });

        currentState = uniqueStateRegistry.get(Role.INITIAL);
        statesReached.put(state, false);
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
        statesReached.remove(state);
        return this;
    }

    /**
     * Called on every frame of the robot, it ticks and manages the state machine.
     */
    public void loop() {
        if (terminated) {
            return;
        }

        /* Sanity. */
        if (statesReached.size() == 0) {
            throw new InsaneStateMachineException("no states in state machine");
        }

        /* Run start() once we reach the state. */
        if (Boolean.FALSE.equals(statesReached.get(currentState))) {
            currentState.start();
            statesReached.put(currentState, true);
        }

        /* Run the current state's looping function. */
        currentState.loop();

        /* Current state state switching logic. */
        for (Edge<?> edge : currentState.getEdges()) {
            if (edge.getCallback().valve(currentState)) {
                boolean stateFound = false;
                for (State state : statesReached.keySet()) {
                    if (state.getClass() == edge.getTo()) {
                        currentState = state;
                        stateFound = true;
                        break;
                    }
                }

                if (stateFound) {
                    /* It's fine to use '==' here, since we're checking for object identity, not
                     * equivalence. Thanks AP CSA! */
                    terminated = uniqueStateRegistry.get(Role.TERMINATING) == currentState;
                } else {
                    throw new UnknownStateException(edge.getTo());
                }
            }
        }
    }

    /**
     * Generate a Graphviz DOT diagram of the currently fixed state machine.
     */
    public String generateDot() {
        StringBuilder str = new StringBuilder("digraph {\ncomment=\"This file was generated by \"" +
                this.getClass().getCanonicalName());

        /* TODO: Support and show diagnostics. */

        /* Styling and whatnot of the vertices/nodes. */
        for (State state : statesReached.keySet()) {
            Meta stateMeta = state.getClass().getAnnotation(Meta.class);

            assert stateMeta != null;
            str.append("\t\"")
                    .append(state.getClass().getSimpleName())
                    .append("\" [color = \"")
                    .append(stateMeta.color())
                    .append("\"]");
        }

        /* Edges between vertices/nodes. */
        for (State state : statesReached.keySet()) {
            for (Edge<?> edge : state.getEdges()) {
                str.append("\t\"")
                        .append(state.getClass().getSimpleName())
                        .append("\" -> \"")
                        .append(edge.getTo().getSimpleName())
                        .append("\"");
            }
        }

        str.append("}");

        return str.toString();
    }
}
