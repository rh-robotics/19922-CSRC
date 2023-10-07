package org.firstinspires.ftc.teamcode.subsystems.fsm;

/**
 * A runtime exception representing a malformed state encountered at state machine construction.
 */
public class MalformedStateException extends RuntimeException {
    public MalformedStateException(Class<?> offender, String reason) {
        super("State '" + offender.getSimpleName() + "' is malformed: " + reason + ".");
    }
}
