package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.fsm.Edge;
import org.firstinspires.ftc.teamcode.subsystems.fsm.State;
import org.firstinspires.ftc.teamcode.subsystems.fsm.StateMachine;

/**
 * Autonomous OpMode to test the FSM subsystem.
 */
@Autonomous(name = "State machine test", group = "Testing OpModes")
public class StateMachineTest extends OpMode {
    public static Telemetry pTelemetry;
    public static Gamepad gamepad;
    private final StateMachine stateMachine = new StateMachine();

    @Override
    public void init() {
        pTelemetry = telemetry;
        gamepad = gamepad1;
        stateMachine.addState(new IdleState()).addState(new ForwardState());
    }

    @Override
    public void loop() {
        stateMachine.loop();
        telemetry.update();
    }
}

/**
 * A state representing moving forward.
 */
class ForwardState implements State {
    @Override
    public Edge<?>[] getEdges() {
        return new Edge[]{
                new Edge<>(IdleState.class, (state) -> StateMachineTest.gamepad.a)
        };
    }

    @Override
    public void loop() {
        StateMachineTest.pTelemetry.addData("state", "ForwardState");
        StateMachineTest.pTelemetry.addData("buttonA", StateMachineTest.gamepad.a);
        StateMachineTest.pTelemetry.addData("buttonB", StateMachineTest.gamepad.b);
    }
}

/**
 * A state representing idelation.
 */
class IdleState implements State {
    @Override
    public Edge<?>[] getEdges() {
        return new Edge[]{
                new Edge<>(ForwardState.class, (state) -> StateMachineTest.gamepad.b)
        };
    }

    @Override
    public void loop() {
        StateMachineTest.pTelemetry.addData("state", "IdleState");
        StateMachineTest.pTelemetry.addData("buttonA", StateMachineTest.gamepad.a);
        StateMachineTest.pTelemetry.addData("buttonB", StateMachineTest.gamepad.b);
    }
}
