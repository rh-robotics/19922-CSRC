package org.firstinspires.ftc.teamcode.subsystems.fsm;

public class Edge {
    /**
     * The state to transition to.
     */
    private final String to;

    /**
     * The callback that decides if the transition should occur.
     */
    private final EdgeCallback callback;

    /** Constructs a new edge.
     * @param to The state to transition to.
     * @param callback The callback that decides if the transition should occur. */
    public Edge(String to, EdgeCallback callback) {
        this.to = to;
        this.callback = callback;
    }

    /**
     * Gets the name of the state to transition to.
     */
    public String getTo() {
        return to;
    }

    /**
     * Gets the callback that decides if the transition should occur.
     */
    public EdgeCallback getCallback() {
        return callback;
    }
}
