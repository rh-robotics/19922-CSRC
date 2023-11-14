package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.HWC;

/***
 * TeleOp OpMode for simply running the passover servos, will be deleted
 */
@TeleOp(name = "ServoTesting", group = "Testing")
public class ServoTesting extends OpMode {
    HWC robot; // Declare the object for HWC, will allow us to access all the motors declared there!

    // init() Runs ONCE after the driver hits initialize
    @Override
    public void init() {
        // Tell the driver the Op is initializing
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        // Initialize the robot hardware
        robot = new HWC(hardwareMap, telemetry);

        // Move servos to position 0
        robot.passoverArmLeft.setPosition(0);
        robot.passoverArmRight.setPosition(0);

        // Tell the driver the robot is ready
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    // init_loop() - Runs continuously until the driver hits play
    @Override
    public void init_loop() {}

    // Start() - Runs ONCE when the driver presses play
    @Override
    public void start() { robot.time.reset(); }

    // loop() - Runs continuously while the OpMode is active
    @Override
    public void loop() {
        double passoverPositionLeft = robot.passoverArmLeft.getPosition();
        double passoverPositionRight = robot.passoverArmRight.getPosition();

        // --------------- Update Servo Position Values --------------- //
        // Dpad up and down control the left servo
        if (gamepad1.dpad_up) {
            passoverPositionLeft += 0.01;
        } else if (gamepad1.dpad_down) {
            passoverPositionLeft -= 0.01;
        }

        // Right Bumper and Trigger control the right servo
        if (gamepad1.right_bumper) {
            passoverPositionRight += 0.01;
        } else if (gamepad1.right_trigger != 0) {
            passoverPositionRight -= 0.01;
        }


        // Run Intake
        if (gamepad1.right_trigger != 0) {
            robot.runIntake(gamepad1.right_trigger);
        }

        // --------------- Move Servos --------------- //
        robot.passoverArmLeft.setPosition(passoverPositionLeft);
        robot.passoverArmRight.setPosition(passoverPositionRight);

        // --------------- Telemetry Updates --------------- //
        telemetry.addData("Status", "Running");
        telemetry.addLine();
        telemetry.addData("Passover Left Position", passoverPositionLeft);
        telemetry.addData("Passover Right Position", passoverPositionRight);
        telemetry.update();
    }
}
