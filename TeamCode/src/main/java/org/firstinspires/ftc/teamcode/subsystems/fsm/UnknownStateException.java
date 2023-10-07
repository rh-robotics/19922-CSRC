package org.firstinspires.ftc.teamcode.subsystems.fsm;

/**
 * A runtime exception representing an unknown state.
 * <br/>
 * TODO: Figure out if this can be collapsed and covered under InsaneStateMachinrException.
 */
public class UnknownStateException extends RuntimeException {
    public UnknownStateException(Class<?> offender) {
        super("Valve function referenced unknown state, '" + offender.getSimpleName() + "'.");
    }
}
