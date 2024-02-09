package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.HWC;
import org.firstinspires.ftc.teamcode.teleop.enums.MultiplierSelection;
import org.firstinspires.ftc.teamcode.teleop.enums.TeleOpState;

/**
 * TeleOp OpMode for simply driving with strafing wheels
 */
@TeleOp(name = "Two Driver", group = "Primary OpModes")
public class TwoDriverTeleOp extends OpMode {
    HWC robot;
    TeleOpState state = TeleOpState.RESTING;
    MultiplierSelection selection = MultiplierSelection.TURN_SPEED;
    boolean testingMode = false;
    double turnSpeed = 0.5; // Speed multiplier for turning
    double driveSpeed = 1; // Speed multiplier for driving
    double strafeSpeed = 0.8; // Speed multiplier for strafing

    @Override
    public void init() {
        // ------ Telemetry ------ //
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        // ------ Initialize Robot Hardware ------ //
        robot = new HWC(hardwareMap, telemetry, false);

        // ------ Reset Servos ------ //
        robot.clawL.setPosition(1);
        robot.clawR.setPosition(1);

        // ------ Telemetry ------ //
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // ------ GamePad Updates ------ //
        robot.previousGamepad1.copy(robot.currentGamepad1);
        robot.previousGamepad2.copy(robot.currentGamepad2);
        robot.currentGamepad1.copy(gamepad1);
        robot.currentGamepad2.copy(gamepad2);

        // ------ Speed Multiplier Selection ------ //
        if (robot.currentGamepad1.a && !robot.previousGamepad1.a) {
            selection = MultiplierSelection.TURN_SPEED;
        } else if (robot.currentGamepad1.b && !robot.previousGamepad1.b) {
            selection = MultiplierSelection.DRIVE_SPEED;
        } else if (robot.currentGamepad1.x && !robot.previousGamepad1.x) {
            selection = MultiplierSelection.STRAFE_SPEED;
        }

        // ------ Speed Multiplier Changes ------ //
        switch (selection) {
            case TURN_SPEED:
                if (robot.currentGamepad1.dpad_up && !robot.previousGamepad1.dpad_up) {
                    turnSpeed += 0.1;
                } else if (robot.currentGamepad1.dpad_down && !robot.previousGamepad1.dpad_down) {
                    turnSpeed -= 0.1;
                }
                break;
            case DRIVE_SPEED:
                if (robot.currentGamepad1.dpad_up && !robot.previousGamepad1.dpad_up) {
                    driveSpeed += 0.1;
                } else if (robot.currentGamepad1.dpad_down && !robot.previousGamepad1.dpad_down) {
                    driveSpeed -= 0.1;
                }
                break;
            case STRAFE_SPEED:
                if (robot.currentGamepad1.dpad_up && !robot.previousGamepad1.dpad_up) {
                    strafeSpeed += 0.1;
                } else if (robot.currentGamepad1.dpad_down && !robot.previousGamepad1.dpad_down) {
                    strafeSpeed -= 0.1;
                }
                break;
        }

        // ------ Enabling Testing Mode ------ //
        if (robot.currentGamepad1.left_bumper && robot.currentGamepad1.right_bumper && !robot.previousGamepad1.left_bumper && !robot.previousGamepad1.right_bumper) {
            testingMode = true;

            telemetry.addData("TESTING MODE IS ENABLED. CONTROLLER OUTPUTS WILL BE SHOWN.", "");
            telemetry.addLine();
        }

        // ------ Telemetry ------ //
        telemetry.addData("Press A to start changing turn speed", "");
        telemetry.addData("Press B to start changing drive speed", "");
        telemetry.addData("Press X to start changing strafe speed", "");
        telemetry.addLine();
        telemetry.addData("Modifying", selection);
        telemetry.addLine();
        telemetry.addData("Turn Speed", turnSpeed);
        telemetry.addData("Drive Speed", driveSpeed);
        telemetry.addData("Strafe Speed", strafeSpeed);
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

        // ------ Power & Controller Values ------ //
        double leftFPower;
        double rightFPower;
        double leftBPower;
        double rightBPower;
        double passoverPosition = robot.passoverArmLeft.getPosition();
        double wristPosition = robot.wrist.getPosition();
        double drive = -robot.currentGamepad1.left_stick_x * driveSpeed;
        double turn = robot.currentGamepad1.left_stick_y * turnSpeed;
        double strafe = -robot.currentGamepad1.right_stick_x * strafeSpeed;

        // ------ Calculate Drive Power ------ //
        if (drive != 0 || turn != 0) {
            state = TeleOpState.DRIVING;
            leftFPower = Range.clip(drive + turn, -1.0, 1.0);
            rightFPower = Range.clip(drive - turn, -1.0, 1.0);
            leftBPower = Range.clip(drive + turn, -1.0, 1.0);
            rightBPower = Range.clip(drive - turn, -1.0, 1.0);
        } else if (strafe != 0) {
            state = TeleOpState.DRIVING;
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
        if (robot.currentGamepad2.right_trigger != 0) {
            robot.runIntake(robot.currentGamepad2.right_trigger);
            state = TeleOpState.INTAKING;
        }
        if (robot.currentGamepad2.left_trigger != 0) {
            robot.runIntake(-robot.currentGamepad2.left_trigger);
            state = TeleOpState.INTAKING;
        }

        // ------ Passover Controls ------ //
        if (robot.currentGamepad2.left_bumper && !robot.previousGamepad2.left_bumper) {
            passoverPosition += 0.05;
        } else if (robot.currentGamepad2.right_bumper && !robot.previousGamepad2.right_bumper) {
            passoverPosition -= 0.05;
        }

        // ------ Wrist Controls ------ //
        if (robot.currentGamepad2.dpad_up && !robot.previousGamepad2.dpad_up) {
            wristPosition += 0.1;
        } else if (robot.currentGamepad2.dpad_down && !robot.previousGamepad2.dpad_down) {
            wristPosition -= 0.1;
        }

        // ------ Claw Controls ------ //
        if (robot.currentGamepad2.x && !robot.previousGamepad2.x) {
            robot.toggleClaw('L');
        }
        if (robot.currentGamepad2.y && !robot.previousGamepad2.y) {
            robot.toggleClaw('R');
        }

        // ------ Run Motors ------ //
        robot.leftFront.setPower(leftFPower);
        robot.leftRear.setPower(leftBPower);
        robot.rightFront.setPower(rightFPower);
        robot.rightRear.setPower(rightBPower);

        // ------ Run Servos ------ //
        robot.passoverArmLeft.setPosition(passoverPosition);
        robot.passoverArmRight.setPosition(passoverPosition);
        robot.wrist.setPosition(wristPosition);

        // ------ State Machine ------ //
        switch (state) {
            case DRIVING:
                telemetry.addData("Robot State", "Driving");
                break;

            case DELIVERING:
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
                state = TeleOpState.UNKNOWN;
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
        telemetry.addData("Intake Motor Power", robot.intakeMotor.getPower());
        telemetry.addData("Slide Pulley Left Position", robot.leftPulley.getCurrentPosition());
        telemetry.addData("Slide Pulley Right Position", robot.rightPulley.getCurrentPosition());
        telemetry.addData("Claw Left Position", robot.clawL.getPosition());
        telemetry.addData("Claw Right Position", robot.clawR.getPosition());
        telemetry.addData("Left Passover Position", robot.passoverArmLeft.getPosition());
        telemetry.addData("Right Passover Position", robot.passoverArmRight.getPosition());
        telemetry.addData("Wrist Position", robot.wrist.getPosition());
        telemetry.addData("> Target Wrist Position", wristPosition);
        telemetry.addData("> Target Passover Position", passoverPosition);
        telemetry.addLine();
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFPower, rightFPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBPower, rightBPower);

        // ------ Testing Mode Telemetry ------ //
        if (testingMode) {
            telemetry.addLine();
            telemetry.addData(">", "Gamepad Information");
            telemetry.addData("Gamepad 1 Left Stick X", robot.currentGamepad1.left_stick_x);
            telemetry.addData("Gamepad 1 Left Stick Y", robot.currentGamepad1.left_stick_y);
            telemetry.addData("Gamepad 1 Right Stick X", robot.currentGamepad1.right_stick_x);
            telemetry.addData("Gamepad 1 Right Stick Y", robot.currentGamepad1.right_stick_y);
            telemetry.addData("Gamepad 1 Left Trigger", robot.currentGamepad1.left_trigger);
            telemetry.addData("Gamepad 1 Right Trigger", robot.currentGamepad1.right_trigger);
            telemetry.addData("Gamepad 1 Left Bumper", robot.currentGamepad1.left_bumper);
            telemetry.addData("Gamepad 1 Right Bumper", robot.currentGamepad1.right_bumper);
            telemetry.addData("Gamepad 1 D-Pad Up", robot.currentGamepad1.dpad_up);
            telemetry.addData("Gamepad 1 D-Pad Down", robot.currentGamepad1.dpad_down);
            telemetry.addData("Gamepad 1 D-Pad Left", robot.currentGamepad1.dpad_left);
            telemetry.addData("Gamepad 1 D-Pad Right", robot.currentGamepad1.dpad_right);
            telemetry.addData("Gamepad 1 A", robot.currentGamepad1.a);
            telemetry.addData("Gamepad 1 B", robot.currentGamepad1.b);
            telemetry.addData("Gamepad 1 X", robot.currentGamepad1.x);
            telemetry.addData("Gamepad 1 Y", robot.currentGamepad1.y);
            telemetry.addData("Gamepad 1 Back", robot.currentGamepad1.back);

            telemetry.addData("Gamepad 2 Left Stick X", robot.currentGamepad2.left_stick_x);
            telemetry.addData("Gamepad 2 Left Stick Y", robot.currentGamepad2.left_stick_y);
            telemetry.addData("Gamepad 2 Right Stick X", robot.currentGamepad2.right_stick_x);
            telemetry.addData("Gamepad 2 Right Stick Y", robot.currentGamepad2.right_stick_y);
            telemetry.addData("Gamepad 2 Left Trigger", robot.currentGamepad2.left_bumper);
            telemetry.addData("Gamepad 2 Right Trigger", robot.currentGamepad2.right_bumper);
            telemetry.addData("Gamepad 2 Left Bumper", robot.currentGamepad2.left_bumper);
            telemetry.addData("Gamepad 2 Right Bumper", robot.currentGamepad2.right_bumper);
            telemetry.addData("Gamepad 2 D-Pad Up", robot.currentGamepad2.dpad_up);
            telemetry.addData("Gamepad 2 D-Pad Down", robot.currentGamepad2.dpad_down);
            telemetry.addData("Gamepad 2 D-Pad Left", robot.currentGamepad2.dpad_left);
            telemetry.addData("Gamepad 2 D-Pad Right", robot.currentGamepad2.dpad_right);
            telemetry.addData("Gamepad 2 A", robot.currentGamepad2.a);
            telemetry.addData("Gamepad 2 B", robot.currentGamepad2.b);
            telemetry.addData("Gamepad 2 X", robot.currentGamepad2.x);
            telemetry.addData("Gamepad 2 Y", robot.currentGamepad2.y);
            telemetry.addData("Gamepad 2 Back", robot.currentGamepad2.back);
        }
        telemetry.update();
    }
}