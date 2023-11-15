package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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

    // --------------- Declare Trajectories --------------- //
    Trajectory LINESSIDE_park, FARSIDE_park;

    // --------------- Declare Variables --------------- //
    boolean parkOnly = true;

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

        // --------------- Declare Trajectories --------------- //
        FARSIDE_park = robot.drive.trajectoryBuilder(new Pose2d(12, -35, Math.toRadians(90)))
                .strafeLeft(40)
                .build();

        LINESSIDE_park = robot.drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(84)
                .build();

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

        // --------------- Update ParkOnly Value --------------- //
        if (gamepad1.left_bumper) {
            parkOnly = true;
        } else if (gamepad1.right_bumper) {
            parkOnly = false;
        }

        // --------------- Update Beacon Position --------------- //
        // TODO: Update beacon position using CV (what does our beacon even look like?!?)

        // --------------- Update Telemetry --------------- //
        telemetry.addData("Press 'A' for red alliance, and press 'B' for blue alliance", "");
        telemetry.addData("Press 'X' for LEFT starting position, Press 'Y' for RIGHT starting position", "");
        telemetry.addData("Press 'LEFT BUMPER' for park only, Press 'RIGHT BUMPER' for full auton", "");
        telemetry.addLine();
        telemetry.addData("Selected Alliance", alliance);
        telemetry.addData("Selected Position", startPosition);
        telemetry.addData("Park Only", parkOnly);
        telemetry.addData("Detected Beacon Position", beaconPosition);
        telemetry.update();
    }

    @Override
    public void loop() {
        // --------------- Run Auton --------------- //
        if (parkOnly) {
            switch(alliance) {
                case RED:
                    switch(startPosition) {
                        case LEFT:
                            robot.drive.followTrajectory(LINESSIDE_park);
                            break;
                        case RIGHT:
                            robot.drive.followTrajectoryAsync(FARSIDE_park);
                            break;
                    }
                    break;
                case BLUE:
                    // TODO: Write blue alliance park only auton
                    break;
            }
        } else {
            // TODO: Write full auton
        }

        // --------------- Update Things --------------- //
        robot.drive.update();

        // --------------- Update Telemetry --------------- //
        telemetry.addData("Status", "Running");
        telemetry.addLine();
        telemetry.addData("Selected Alliance", alliance);
        telemetry.addData("Selected Position", startPosition);
        telemetry.addData("Park Only", parkOnly);
        telemetry.addData("Detected Beacon Position", beaconPosition);
        telemetry.addLine();
        telemetry.addData("Robot Pose", robot.drive.getPoseEstimate());
        telemetry.addData("Robot Heading", robot.drive.getRawExternalHeading());
        telemetry.update();
    }
}