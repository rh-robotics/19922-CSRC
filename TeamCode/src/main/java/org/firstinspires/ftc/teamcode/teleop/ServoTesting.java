package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.HWC;

/***
 * TeleOp for simply running the passover servos, will be deleted
 */
enum ServoControl {
    LEFT,
    RIGHT
}

@TeleOp(name = "ServoTesting", group = "Testing")
public class ServoTesting extends OpMode {
    HWC robot; // Robot Hardware Class - Contains all hardware devices & related methods
    ServoControl servoControl = ServoControl.LEFT;

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

        // --------------- Update Which Servo Is Being Controlled --------------- //
        if(gamepad1.a) { servoControl = ServoControl.LEFT; }
        else if(gamepad1.b) { servoControl = ServoControl.RIGHT; }

        // --------------- Update Servo Position Values --------------- //
        switch(servoControl) {
            case LEFT:
                if(gamepad1.dpad_up) { passoverPositionLeft += 0.01; }
                else if(gamepad1.dpad_down) { passoverPositionLeft -= 0.01; }
                break;
            case RIGHT:
                if(gamepad1.dpad_up) { passoverPositionRight += 0.01; }
                else if(gamepad1.dpad_down) { passoverPositionRight -= 0.01; }
                break;
        }

        // --------------- Move Servos --------------- //
        robot.passoverArmLeft.setPosition(passoverPositionLeft);
        robot.passoverArmRight.setPosition(passoverPositionRight);

        // --------------- Telemetry Updates --------------- //
        telemetry.addData("Status", "Running");
        telemetry.addLine();
        telemetry.addData("Press 'A' to control left servo", "");
        telemetry.addData("Press 'B' to control right servo", "");
        telemetry.addData("Press 'DPAD_UP'/'DPAD_DOWN' to increase/decrease servo target position", "");
        telemetry.addLine();
        telemetry.addData("Controlling Servo", servoControl);
        telemetry.addData("Passover Left Target Position", passoverPositionLeft);
        telemetry.addData("Passover Right Target Position", passoverPositionRight);
        telemetry.addData("Passover Left Actual Position", robot.passoverArmLeft.getPosition());
        telemetry.addData("Passover Right Actual Position", robot.passoverArmRight.getPosition());
        telemetry.update();
    }
}
