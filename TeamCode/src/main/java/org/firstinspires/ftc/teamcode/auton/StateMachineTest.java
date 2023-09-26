package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.fsm.Edge;
import org.firstinspires.ftc.teamcode.subsystems.fsm.State;
import org.firstinspires.ftc.teamcode.subsystems.fsm.StateMachine;

/**
 * Autonomous OpMode to test the FSM subsystem.
 */
@Autonomous(name = "Subsystem test", group = "Testing OpModes")
public class StateMachineTest extends LinearOpMode {
    StateMachine stateMachine;

    @Override
    public void runOpMode() {
        stateMachine.addState(new IdleState()).addState(new ForwardState());

        while (opModeIsActive()) {
            stateMachine.loop();
        }
    }
}

/**
 * A state representing moving forward.
 */
class ForwardState implements State {
    Edge<?>[] edges = new Edge[]{
        new Edge<>(IdleState.class, (state) -> {
            // You can replace this with any logic that, when it returns true, will switch
            // the state machine over to IdleState.
            return false;
        })
    };
}

/**
 * A state representing idelation.
 */
class IdleState implements State {
    Edge<?>[] edges = new Edge[]{
            new Edge<>(ForwardState.class, (state) -> {
                // You can replace this with any logic that, when it returns true, will switch
                // the state machine over to ForwardState.
                return false;
            })
    };
}
