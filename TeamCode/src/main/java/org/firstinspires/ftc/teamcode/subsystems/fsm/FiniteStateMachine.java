package org.firstinspires.ftc.teamcode.subsystems.fsm;

import androidx.annotation.NonNull;

import java.util.HashMap;

/**
 * Represents a finite state machine.
 */
public class FiniteStateMachine {
    /**
     * The flat list of states.
     */
    private final HashMap<String, MetaState> states;

    /**
     * The current state being executed.
     */
    private MetaState current;

    /**
     * Constructs a new finite state machine, from a builder. This is the only way!!!
     * @param builder The builder object that contains the state machine specification.
     */
    public FiniteStateMachine(@NonNull FiniteStateMachineBuilder builder) {
        assert(builder.getStates().size() != 0);

        states = new HashMap<>();
        builder.getStates().forEach((state) -> {
            state.init();
            states.put(state.getName(), new MetaState(state, StateState.UNRAN));
        });

        current = new MetaState(builder.getStates().get(0), StateState.UNRAN);
    }

    /** Called on every frame of the robot, it ticks and manages the state machine. */
    public void loop() {
        states.forEach((k, v) -> {
            if (v.getStateState() == StateState.UNRAN) {
                v.getState().initLoop();
            } else {
                v.getState().loop();
            }
        });

        if (current.getStateState() == StateState.UNRAN) {
            current.getState().start();
        }

        current.getState().loop();

        // TODO: Handle multiple true valves instead of blindly accepting the first.
        for (Edge edge : current.getState().getEdges()) {
            if (edge.getCallback().valve(current.getState())) {
                current = states.get(edge.getTo());
                break;
            }
        }
    }
}

// TODO: Figure out if @isaccbarker should remove this code, and
//  admire how simple this un-flattening code is.
// Create a graph from the list of states.
// final EdgeCallback[][] adjacencyMatrix;
// ...
// adjacencyMatrix = new EdgeCallback[states.size()][states.size()];
/* for (x = 0; x < states.size(); x++) {
    Edge[] edges = states.get(x).getEdges();

    for (i = 0; i < edges.length; i++) {
        //noinspection StatementWithEmptyBody
        while (states.get(y++).getName().equals(edges[i].getTo()));
        adjacencyMatrix[x][y] = edges[y].getCallback();
    }
} */
