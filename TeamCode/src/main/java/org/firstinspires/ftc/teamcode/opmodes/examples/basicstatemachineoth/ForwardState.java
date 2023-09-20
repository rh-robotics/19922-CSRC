package org.firstinspires.ftc.teamcode.opmodes.examples.basicstatemachineoth;

import org.firstinspires.ftc.teamcode.subsystems.fsm.Edge;
import org.firstinspires.ftc.teamcode.subsystems.fsm.State;

public class ForwardState implements State {
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
