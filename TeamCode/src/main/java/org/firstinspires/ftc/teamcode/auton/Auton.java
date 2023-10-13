package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.HWC;
import org.firstinspires.ftc.teamcode.subsystems.fsm.Edge;
import org.firstinspires.ftc.teamcode.subsystems.fsm.State;
import org.firstinspires.ftc.teamcode.subsystems.fsm.StateMachine;

/**
 * Main Autonomous OpMode for the robot
 */
@Autonomous
public class Auton extends OpMode {
    private final StateMachine stateMachine = new StateMachine();

    @Override
    public void init() {
        HWC robot = new HWC(hardwareMap, telemetry); // Declare HardwareClass Object
        stateMachine
                .addState(new ID2B())
                .addState(new DPP())
                .addState(new D2BB())
                .addState(new SOB())
                .addState(new D2PS())
                .addState(new PP())
                .addState(new PARK());
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {}

    @Override
    public void loop() {

    }
}

/**
 * Initial Driving to Backboard
 */
@State.Meta(role = State.Role.INITIAL, color = "#335C67")
class ID2B implements State {
    @Override
    public Edge<?>[] getEdges() {
        return new Edge[]{

        };
    }

    @Override
    public void loop() {

    }
}
@State.Meta(color = "#335C67")
class DPP implements State {
    @Override
    public Edge<?>[] getEdges() {
        return new Edge[]{

        };
    }

    @Override
    public void loop() {

    }
}

@State.Meta(color = "#335C67")
class D2BB implements State {
    @Override
    public Edge<?>[] getEdges() {
        return new Edge[]{

        };
    }

    @Override
    public void loop() {

    }
}

@State.Meta(color = "#335C67")
class SOB implements State {
    @Override
    public Edge<?>[] getEdges() {
        return new Edge[]{

        };
    }

    @Override
    public void loop() {

    }
}

@State.Meta(color = "#335C67")
class D2PS implements State {
    @Override
    public Edge<?>[] getEdges() {
        return new Edge[]{

        };
    }

    @Override
    public void loop() {

    }
}

@State.Meta(color = "#335C67")
class PP implements State {
    @Override
    public Edge<?>[] getEdges() {
        return new Edge[]{

        };
    }

    @Override
    public void loop() {

    }
}

@State.Meta(color = "#335C67")
class PARK implements State {
    @Override
    public Edge<?>[] getEdges() {
        return new Edge[]{

        };
    }

    @Override
    public void loop() {

    }
}