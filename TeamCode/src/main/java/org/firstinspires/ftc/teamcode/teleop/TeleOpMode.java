package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.robot.RobotState;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.HWC;

/**
 * Enum representing which speed to change in init_loop()
 */
enum MultiplierSelection {
    TURN_SPEED,
    DRIVE_SPEED,
    STRAFE_SPEED
}

/**
 * TeleOp OpMode for simply driving with strafing wheels
 */
@TeleOp(name = "TeleOp", group = "Iterative OpMode")
public class TeleOpMode extends OpMode {
    HWC robot; // Declare the object for HWC, will allow us to access all the motors declared there!
    RobotState state = RobotState.RESTING;
    double turnSpeed = 0.5; // Speed multiplier for turning
    double driveSpeed = 1; // Speed multiplier for driving
    double strafeSpeed = 0.8; // Speed multiplier for strafing
    MultiplierSelection selection = MultiplierSelection.TURN_SPEED; // Enum for selecting which speed to change

    // ------ GamePads ------ //
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    // init() Runs ONCE after the driver hits initialize
    @Override
    public void init() {
        // Tell the driver the Op is initializing
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        // Initialize the robot hardware
        robot = new HWC(hardwareMap, telemetry);

        // Tell the driver the robot is ready
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    // init_loop() - Runs continuously until the driver hits play
    @Override
    public void init_loop() {
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        // Reset Servos to Position 0
        robot.clawL.setPosition(1);
        robot.clawR.setPosition(1);

        // Select which speed to change
        if (currentGamepad1.a && !previousGamepad1.a) {
            selection = MultiplierSelection.TURN_SPEED;
        } else if (currentGamepad1.b && !previousGamepad1.b) {
            selection = MultiplierSelection.DRIVE_SPEED;
        } else if (currentGamepad1.x && !previousGamepad1.x) {
            selection = MultiplierSelection.STRAFE_SPEED;
        }

        // Change the speed multiplier for selection
        switch (selection) {
            case TURN_SPEED:
                if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                    turnSpeed += 0.1;
                } else if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                    turnSpeed -= 0.1;
                }
                break;
            case DRIVE_SPEED:
                if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                    driveSpeed += 0.1;
                } else if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                    driveSpeed -= 0.1;
                }
                break;
            case STRAFE_SPEED:
                if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                    strafeSpeed += 0.1;
                } else if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                    strafeSpeed -= 0.1;
                }
                break;
        }

        // Tell driver the commands
        telemetry.addData("Press A to start changing turn speed", "");
        telemetry.addData("Press B to start changing drive speed", "");
        telemetry.addData("Press X to start changing strafe speed", "");

        // Divider Line
        telemetry.addLine();

        // Display the current modifiers
        telemetry.addData("Modifying", selection);
        telemetry.addData("Turn Speed", turnSpeed);
        telemetry.addData("Drive Speed", driveSpeed);
        telemetry.addData("Strafe Speed", strafeSpeed);

        // Update telemetry
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
        // ------ GamePad Updates ------ //
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        // ------ Power & Controller Values ------ //
        double leftFPower;
        double rightFPower;
        double leftBPower;
        double rightBPower;
        double drive = -currentGamepad1.left_stick_x * driveSpeed;
        double turn = currentGamepad1.left_stick_y * turnSpeed;
        double strafe = -currentGamepad1.right_stick_x * strafeSpeed;

        // ------ Calculate Drive Power ------ //
        if (drive != 0 || turn != 0) {
            state = RobotState.DRIVING;
            leftFPower = Range.clip(drive + turn, -1.0, 1.0);
            rightFPower = Range.clip(drive - turn, -1.0, 1.0);
            leftBPower = Range.clip(drive + turn, -1.0, 1.0);
            rightBPower = Range.clip(drive - turn, -1.0, 1.0);
        } else if (strafe != 0) {
            // Strafing
            state = RobotState.DRIVING;
            leftFPower = strafe;
            rightFPower = strafe;
            leftBPower = -strafe;
            rightBPower = -strafe;
        } else {
            leftFPower = 0;
            rightFPower = 0;
            leftBPower = 0;
            rightBPower = 0;
        }

        // ------ Intake Controls ------ //
        if (currentGamepad2.right_trigger != 0) {
            robot.runIntake(currentGamepad2.right_trigger);
            state = RobotState.INTAKING;
        }
        if (currentGamepad2.left_trigger != 0) {
            robot.runIntake(-currentGamepad2.left_trigger);
            state = RobotState.INTAKING;
        }

        // ------ Passover Controls ------ //
        if (currentGamepad2.right_stick_y != 0) {
            robot.manualArm(-currentGamepad2.right_stick_y);
        } else {
            robot.manualArm(0);
        }

        // ------ Claw Controls ------ //
        if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
            robot.toggleClaw('L');
        }
        if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
            robot.toggleClaw('R');
        }

        // ------ Run Motors ------ //
        robot.leftFront.setPower(leftFPower);
        robot.leftRear.setPower(leftBPower);
        robot.rightFront.setPower(rightFPower);
        robot.rightRear.setPower(rightBPower);
        robot.passoverArmLeft.setPower(-currentGamepad2.left_stick_y);
        robot.passoverArmRight.setPower(-currentGamepad2.left_stick_y);

        // ------ State Machine ------ //
        switch (state) {
            case DRIVING:
                telemetry.addData("Robot State", "Driving");
                break;

            case DELIVERYING:
                telemetry.addData("Robot State", "Delivering");
                break;

            case INTAKING:
                telemetry.addData("Robot State", "Intake");
                break;

            case RESTING:
                telemetry.addData("Robot State", "Resting");
                break;

            case UNKNOWN:
                telemetry.addData("Robot State", "UNKNOWN");
                break;

            default:
                telemetry.addData("Robot State", "UNKNOWN");
                state = RobotState.UNKNOWN;
                break;
        }

        // ------ Telemetry Updates ------ //
        telemetry.addData("Status", "Running");
        telemetry.addData("Robot State", state);
        telemetry.addLine();
        telemetry.addData("Gamepad 1 Left Stick X", "Driving Forward / Backward");
        telemetry.addData("Gamepad 1 Left Stick Y", "Turning");
        telemetry.addData("Gamepad 1 Right Stick X", "Strafing");
        telemetry.addData("Gamepad 2 Left Stick Y", "Passover Control");
        telemetry.addData("Gamepad 2 Right Stick Y", "Arm Control");
        telemetry.addData("Gamepad 2 Left Bumper", "Toggle Claw Left");
        telemetry.addData("Gamepad 2 Right Bumper", "Toggle Claw Right");
        telemetry.addData("Gamepad 2 Left Trigger", "Intake Out");
        telemetry.addData("Gamepad 2 Right Trigger", "Intake In");
        telemetry.addLine();
        telemetry.addData("Claw Left Position", robot.clawL.getPosition());
        telemetry.addData("Claw Right Position", robot.clawR.getPosition());
        telemetry.addData("Left Passover Power", robot.passoverArmLeft.getPower());
        telemetry.addData("Right Passover Power", robot.passoverArmRight.getPower());
        telemetry.addLine();
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFPower, rightFPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBPower, rightBPower);
        telemetry.addLine();
        telemetry.addData("Gamepad 2 Left Trigger", currentGamepad2.left_bumper);
        telemetry.addData("Gamepad 2 Right Trigger", currentGamepad2.right_bumper);

        telemetry.update();
    }

    public enum RobotState {
        DRIVING,
        INTAKING,
        DELIVERYING,
        RESTING,
        UNKNOWN
    }
}