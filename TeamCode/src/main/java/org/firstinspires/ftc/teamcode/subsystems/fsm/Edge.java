package org.firstinspires.ftc.teamcode.subsystems.fsm;

/** An edge in the state machine graph. */
public class Edge<T> {
    /**
     * The state to transition to.
     */
    private final Class<T> to;

    /**
     * The callback that decides if the transition should occur.
     */
    private final EdgeCallback callback;

    /** Constructs a new edge.
     * @param to The state to transition to.
     * @param callback The callback that decides if the transition should occur. */
    public Edge(Class<T> to, EdgeCallback callback) {
        this.to = to;
        this.callback = callback;
    }

    /**
     * Gets the name of the state to transition to.
     * @return Returns the state to transition to.
     */
    public Class<T> getTo() {
        return to;
    }

    /**
     * Gets the callback that decides if the transition should occur.
     * @return Returns the callback.
     */
    public EdgeCallback getCallback() {
        return callback;
    }
}
