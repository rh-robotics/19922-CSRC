package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.HWC;

/**
 * TeleOp OpMode for simply driving with strafing wheels
 */
@TeleOp(name = "Distance test", group = "Primary OpModes")
public class DistanceTest extends OpMode {

    // ------ Declare Others ------ //
    private HWC robot;

    @Override
    public void init() {
        // ------ Telemetry ------ //
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        // ------ Initialize Robot Hardware ------ //
        robot = new HWC(hardwareMap, telemetry, false);

        // ------ Telemetry ------ //
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        robot.time.reset();
    }

    @Override
    public void loop() {
        // ------ GamePad Updates ------ //
        robot.previousGamepad1.copy(robot.currentGamepad1);
        robot.previousGamepad2.copy(robot.currentGamepad2);
        robot.currentGamepad1.copy(gamepad1);
        robot.currentGamepad2.copy(gamepad2);

        if (gamepad1.dpad_left) {
            robot.alignWithBackboard(10);
        }
        if (gamepad1.dpad_right) {
            telemetry.addData("Left Distance", robot.distanceLeft.getDistance(DistanceUnit.CM));
            telemetry.addData("Right Distance", robot.distanceRight.getDistance(DistanceUnit.CM));
        }
    }
}