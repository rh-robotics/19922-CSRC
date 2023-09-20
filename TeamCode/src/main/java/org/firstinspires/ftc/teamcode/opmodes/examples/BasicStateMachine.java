package org.firstinspires.ftc.teamcode.opmodes.examples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.fsm.FiniteStateMachine;
import org.firstinspires.ftc.teamcode.subsystems.fsm.FiniteStateMachineBuilder;

import org.firstinspires.ftc.teamcode.subsystems.fsm.Edge;
import org.firstinspires.ftc.teamcode.subsystems.fsm.State;

/**
 * A state representing moving forward.
 */
class ForwardState implements State {
    @Override
    public Edge[] getEdges() {
        return new Edge[]{
                new Edge("IdleState", (state) -> {
                    // You can replace this with any logic that, when it returns true, will switch
                    // the state machine over to IdleState.
                    return false;
                })
        };
    }

    @Override
    public String getName() {
        return "ForwardState";
    }
}

/**
 * A state representing idelation.
 */
class IdleState implements State {
    @Override
    public Edge[] getEdges() {
        return new Edge[]{
                new Edge("ForwardState", (state) -> {
                    // You can replace this with any logic that, when it returns true, will switch
                    // the state machine over to ForwardState.
                    return false;
                })
        };
    }

    @Override
    public String getName() {
        return "IdleState";
    }
}

/**
 * A basic state machine example, with no robot I/O.
 */
@TeleOp(name = "Basic State Machine", group = "Iterative OpMode")
public class BasicStateMachine extends OpMode {
    /**
     * The state machine that powers this whole damn ordeal!
     */
    private FiniteStateMachine finiteStateMachine;

    /** Runs ONCE after the driver hits initialize.
     * <p>
     * Sets up the robot physically, digitally, and mentally for the trials ahead.
     */
    @Override
    public void init() {
        // Tell the driver the Op is initializing.
        telemetry.addData("Status", "Initializing");

        // Build the state machine.
        FiniteStateMachineBuilder finiteStateMachineBuilder = new FiniteStateMachineBuilder()
                .addState(new ForwardState())
                .addState(new IdleState());
        finiteStateMachine = new FiniteStateMachine(finiteStateMachineBuilder);

        // Tell the driver the robot is ready.
        telemetry.addData("Status", "Initialized");
    }

    /**
     * Code that is called once per frame. Process input, send output to motors, etc.
     * Do it all here!
     */
    @Override
    public void loop() {
        // Call into the state machine and let it do the magic for you!
        finiteStateMachine.loop();
    }
}
