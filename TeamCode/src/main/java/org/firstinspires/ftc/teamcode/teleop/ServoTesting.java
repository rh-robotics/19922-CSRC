package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.HWC;


enum ServoControl {
    PASSOVER_LEFT,
    PASSOVER_RIGHT,
    WRIST_LEFT,
    WRIST_RIGHT,
    CLAW_LEFT,
    CLAW_RIGHT
}

/***
 * TeleOp for simply running the passover servos, will be deleted
 */
@TeleOp(name = "ServoTesting", group = "Testing")
public class ServoTesting extends OpMode {
    HWC robot;
    ServoControl servoControl = ServoControl.PASSOVER_LEFT;
    final double SERVO_SPEED = 0.001;
    double servoPosition;

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
        robot.clawL.setPosition(0);
        robot.clawR.setPosition(0);
        robot.wristL.setPosition(0);
        robot.wristR.setPosition(0);

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
        switch (servoControl) {
            case PASSOVER_LEFT:
                servoPosition = robot.passoverArmLeft.getPosition();
                break;
            case PASSOVER_RIGHT:
                servoPosition = robot.passoverArmRight.getPosition();
                break;
            case WRIST_LEFT:
                servoPosition = robot.wristL.getPosition();
                break;
            case WRIST_RIGHT:
                servoPosition = robot.wristR.getPosition();
                break;
            case CLAW_LEFT:
                servoPosition = robot.clawL.getPosition();
                break;
            case CLAW_RIGHT:
                servoPosition = robot.clawR.getPosition();
                break;
        }


        // --------------- Update Which Servo Is Being Controlled --------------- //
        if (gamepad1.left_bumper && gamepad1.a)
            servoControl = ServoControl.PASSOVER_LEFT;
        else if (gamepad1.right_bumper && gamepad1.a)
            servoControl = ServoControl.PASSOVER_RIGHT;
        else if (gamepad1.left_bumper && gamepad1.b)
            servoControl = ServoControl.WRIST_LEFT;
        else if (gamepad1.right_bumper && gamepad1.b)
            servoControl = ServoControl.WRIST_RIGHT;
        else if (gamepad1.left_bumper && gamepad1.x)
            servoControl = ServoControl.CLAW_LEFT;
        else if (gamepad1.right_bumper && gamepad1.x)
            servoControl = ServoControl.CLAW_RIGHT;

        if (gamepad1.left_bumper || gamepad1.right_bumper || gamepad1.a || gamepad1.b || gamepad1.x || gamepad1.y)
            servoPosition = servoPosition = 0;

        // --------------- Update Servo Position Values --------------- //
        if (gamepad1.dpad_up) {
            servoPosition += SERVO_SPEED;
        } else if (gamepad1.dpad_down) {
            servoPosition -= SERVO_SPEED;
        }

        // --------------- Move Servos --------------- //
        switch (servoControl) {
            case PASSOVER_LEFT:
                robot.passoverArmLeft.setPosition(servoPosition);
                break;
            case PASSOVER_RIGHT:
                robot.passoverArmRight.setPosition(servoPosition);
                break;
            case WRIST_LEFT:
                robot.wristL.setPosition(servoPosition);
                break;
            case WRIST_RIGHT:
                robot.wristR.setPosition(servoPosition);
                break;
            case CLAW_LEFT:
                robot.clawL.setPosition(servoPosition);
                break;
            case CLAW_RIGHT:
                robot.clawR.setPosition(servoPosition);
                break;
        }

        // --------------- Telemetry Updates --------------- //
        telemetry.addData("Status", "Running");
        telemetry.addLine();
        telemetry.addData("Press 'Left Bumper' and 'A' to control PASSOVER_LEFT", "");
        telemetry.addData("Press 'Right Bumper' and 'A' to control PASSOVER_RIGHT", "");
        telemetry.addData("Press 'Left Bumper' and 'B' to control WRIST_LEFT", "");
        telemetry.addData("Press 'Right Bumper' and 'B' to control WRIST_RIGHT", "");
        telemetry.addData("Press 'Left Bumper' and 'X' to control CLAW_LEFT", "");
        telemetry.addData("Press 'Right Bumper' and 'X' to control CLAW_RIGHT", "");
        telemetry.addData("Press 'DPAD_UP'/'DPAD_DOWN' to increase/decrease servo(s) target position", "");
        telemetry.addLine();
        telemetry.addData("Controlling Servo", servoControl);
        telemetry.addData("Servo Target Position", servoPosition);
        telemetry.addLine();
        telemetry.addData("Passover Left Actual Position", robot.passoverArmLeft.getPosition());
        telemetry.addData("Passover Right Actual Position", robot.passoverArmRight.getPosition());
        telemetry.addData("Wrist Left Actual Position", robot.wristL.getPosition());
        telemetry.addData("Wrist Right Actual Position", robot.wristR.getPosition());
        telemetry.addData("Claw Left Actual Position", robot.clawL.getPosition());
        telemetry.addData("Claw Right Actual Position", robot.clawR.getPosition());
        telemetry.update();
    }
}
