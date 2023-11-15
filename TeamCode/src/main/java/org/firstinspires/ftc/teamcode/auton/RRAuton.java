package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.HWC;

enum Alliance {
    RED,
    BLUE
}

enum StartPosition {
    LEFT,
    RIGHT
}

enum BeaconPosition {
    LEFT,
    RIGHT,
    CENTER
}

@Autonomous
@Disabled
public class RRAuton extends OpMode {
    // --------------- Declare Objects --------------- //
    HWC robot;
    Alliance alliance;
    StartPosition startPosition;
    BeaconPosition beaconPosition;

    @Override
    public void init() {
        // --------------- Update Telemetry --------------- //
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        // --------------- Initialize Objects (and default values) --------------- //
        robot = new HWC(hardwareMap, telemetry);
        alliance = Alliance.RED;
        startPosition = StartPosition.LEFT;
        beaconPosition = BeaconPosition.LEFT;

        // --------------- Update Telemetry --------------- //
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // --------------- Update Alliance --------------- //
        if (gamepad1.a) {
            alliance = Alliance.RED;
        } else if (gamepad1.b) {
            alliance = Alliance.BLUE;
        }

        // --------------- Update Start Position --------------- //
        if (gamepad1.x) {
            startPosition = StartPosition.LEFT;
        } else if (gamepad1.y) {
            startPosition = StartPosition.RIGHT;
        }

        // --------------- Update Beacon Position --------------- //
        // TODO: Update beacon position using CV (what does our beacon even look like?!?)

        // --------------- Update Telemetry --------------- //
        telemetry.addData("Press 'A' for red alliance, and press 'B' for blue alliance");
        telemetry.addData("Press 'X' for LEFT starting position, Press 'Y' for RIGHT starting position");
        telemetry.addLine();
        telemetry.addData("Selected Alliance", alliance);
        telemetry.addData("Selected Position", startPosition);
        telemetry.addData("Detected Beacon Position", beaconPosition);
        telemetry.update();
    }

    @Override
    public void loop() {
        // --------------- Run Auton --------------- //
        // TODO: Write auton & trajectories

        // --------------- Update Telemetry --------------- //
        telemetry.addData("Status", "Running");
        telemetry.update();
    }
}