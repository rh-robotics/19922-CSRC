package org.firstinspires.ftc.teamcode.subsystems.fsm;

/**
 * A runtime exception representing an error that occurred during state machine construction, not
 * runtime.
 * ==*/
public class StateMachineConstructionException extends RuntimeException {
    public StateMachineConstructionException(Class<?> offender, String reason) {
        super("State machine construction is invalid, caused by state '" +
                offender.getSimpleName() + "': " + reason + ".");
    }
}
