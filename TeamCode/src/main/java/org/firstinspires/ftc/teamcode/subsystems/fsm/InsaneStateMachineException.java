package org.firstinspires.ftc.teamcode.subsystems.fsm;

/**
 * A runtime exception representing an insane state machine configuration encountered at runtime.
 */
public class InsaneStateMachineException extends RuntimeException {
    public InsaneStateMachineException(String reason) {
        super("Encountered an insane state machine: " + reason + ".");
    }
}
