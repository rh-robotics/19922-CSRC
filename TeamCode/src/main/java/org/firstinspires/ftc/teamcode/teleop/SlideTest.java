package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.HWC;

@TeleOp(name = "slide test", group = "Testing")
public class SlideTest extends OpMode {
    HWC robot;
    // init() Runs ONCE after the driver hits initialize
    @Override
    public void init() {
        // Tell the driver the Op is initializing
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        // Initialize the robot hardware
        robot = new HWC(hardwareMap, telemetry);

        // Move servos to position 0

        // Tell the driver the robot is ready
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    // Start() - Runs ONCE when the driver presses play
    @Override
    public void start() {
        robot.time.reset();
    }

    // loop() - Runs continuously while the OpMode is active
    @Override
    public void loop() {

        robot.rightPulley.setPower(gamepad1.left_stick_y);



        // --------------- Telemetry Updates --------------- //
        telemetry.addData("Pulley Power", robot.rightPulley.getPower());
        telemetry.addData("Pulley Position", robot.rightPulley.getCurrentPosition());
        telemetry.update();
    }
}
