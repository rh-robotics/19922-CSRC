package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.HWC;
import org.firstinspires.ftc.teamcode.teleop.enums.MultiplierSelection;

/**
 * TeleOp OpMode for simply driving with strafing wheels
 */
@TeleOp(name = "Single Driver", group = "Primary OpModes")
public class SingleDriverTeleOp extends OpMode {
    // ------ Intake State Enum ------ //
    private enum IntakeState {INTAKE, OFF, OUTTAKE}

    // ------ Declare Others ------ //
    private HWC robot;
    private MultiplierSelection selection = MultiplierSelection.TURN_SPEED;
    private IntakeState intakeState = IntakeState.OFF;
    private boolean testingMode = false;
    private double turnSpeed = 0.5; // Speed multiplier for turning
    private double driveSpeed = 1; // Speed multiplier for driving
    private double strafeSpeed = 0.8; // Speed multiplier for strafing

    @Override
    public void init() {
        // ------ Telemetry ------ //
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        // ------ Initialize Robot Hardware ------ //
        robot = new HWC(hardwareMap, telemetry, false);

        // ------ Reset Servos ------ //
        robot.clawL.setPosition(1);
        robot.clawR.setPosition(0);
        robot.wrist.setPosition(HWC.wristIntakePos);
        robot.passoverArmLeft.setPosition(HWC.passoverIntakePos);

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
        if ((robot.currentGamepad1.right_bumper && !robot.previousGamepad1.right_bumper) && (robot.currentGamepad1.right_bumper && !robot.previousGamepad1.right_bumper)) {
            testingMode = !testingMode;
        }

        // ------ Telemetry ------ //
        if (testingMode) {
            telemetry.addData("TESTING MODE IS ENABLED. CONTROLLER OUTPUTS WILL BE SHOWN.", "");
            telemetry.addLine();
        }

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
        double turn = robot.currentGamepad1.left_stick_y * driveSpeed;
        double drive = (robot.currentGamepad1.left_trigger - robot.currentGamepad1.right_trigger) * turnSpeed;
        double strafe = -robot.currentGamepad1.left_stick_x * strafeSpeed;

        // ------ Calculate Drive Power ------ //
        if (drive != 0 || turn != 0) {
            leftFPower = Range.clip(drive + turn, -1.0, 1.0);
            rightFPower = Range.clip(drive - turn, -1.0, 1.0);
            leftBPower = Range.clip(drive + turn, -1.0, 1.0);
            rightBPower = Range.clip(drive - turn, -1.0, 1.0);
        } else if (strafe != 0) {
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

        // ------ (GAMEPAD 1) Claw Controls ------ //
        if (robot.currentGamepad1.x && !robot.previousGamepad1.x) {
            robot.toggleClaw('L');
        }
        if (robot.currentGamepad1.y && !robot.previousGamepad1.y) {
            robot.toggleClaw('R');
        }

        // ------ (GAMEPAD 1) Intake Toggle Controls ------ //
        if (robot.currentGamepad1.right_bumper && !robot.previousGamepad1.right_bumper) {
            if (intakeState == IntakeState.INTAKE || intakeState == IntakeState.OUTTAKE) { intakeState = IntakeState.OFF; }
            else if (intakeState == IntakeState.OFF) { intakeState = IntakeState.INTAKE; }
        }

        if (robot.currentGamepad1.left_bumper && !robot.previousGamepad1.left_bumper) {
            if (intakeState == IntakeState.INTAKE || intakeState == IntakeState.OUTTAKE) {
                intakeState = IntakeState.OFF;
            } else if (intakeState == IntakeState.OFF) { intakeState = IntakeState.OUTTAKE; }
        }
        // ------ (GAMEPAD 1) MANUAL Wrist Controls ------ //
        if (robot.currentGamepad1.dpad_up && !robot.previousGamepad1.dpad_up) {
            wristPosition += 0.05;
        } else if (robot.currentGamepad1.dpad_down && !robot.previousGamepad1.dpad_down) {
            wristPosition -= 0.05;
        }

        // ------ (GAMEPAD 1) Position Controls ------ //
        if (robot.currentGamepad1.b && !robot.previousGamepad1.b) {
            wristPosition = HWC.wristDeliveryPos;
            passoverPosition = HWC.passoverDeliveryPos;
        } else if (robot.currentGamepad1.a && !robot.previousGamepad1.a) {
            wristPosition = HWC.wristIntakePos;
            passoverPosition = HWC.passoverIntakePos;
        }

        // ------ (GAMEPAD 1) Slide Controls ------ //
        robot.powerSlides(-robot.currentGamepad1.right_stick_y);

        // ------ (GAMEPAD 2) MANUAL Passover Controls ------ //
        if (robot.currentGamepad2.right_bumper && !robot.previousGamepad2.right_bumper) {
            passoverPosition += 0.05;
        } else if (robot.currentGamepad2.left_bumper && !robot.previousGamepad2.left_bumper) {
            passoverPosition -= 0.05;
        }

        // ------ Run Motors ------ //
        robot.leftFront.setPower(leftFPower);
        robot.leftRear.setPower(leftBPower);
        robot.rightFront.setPower(rightFPower);
        robot.rightRear.setPower(rightBPower);

        // ------ Run Servos ------ //
        robot.passoverArmLeft.setPosition(passoverPosition);
//        robot.passoverArmRight.setPosition(passoverPosition);
        robot.wrist.setPosition(wristPosition);

        // -------- Check Intake State & Run Intake ------ //
        switch(intakeState) {
            case INTAKE:
                robot.intakeMotor.setPower(-1);
                break;
            case OFF:
                robot.intakeMotor.setPower(robot.currentGamepad2.right_stick_x);
                break;
            case OUTTAKE:
                robot.intakeMotor.setPower(1);
                break;
        }

        // ------ Telemetry Updates ------ //
        telemetry.addData("Status", "Running");
        telemetry.addData("Intake State", intakeState);
        telemetry.addLine();
        telemetry.addData(">", "ALL PRIMARY CONTROLS ARE ON GAMEPAD 1");
        telemetry.addData("Left Stick X", "Strafing Left / Right");
        telemetry.addData("Left Stick Y", "Driving Forward / Backward");
        telemetry.addData("Left Trigger / Right Trigger", "Turning Left / Right");
        telemetry.addData("Left Bumper", "Outtake");
        telemetry.addData("Right Bumper", "Intake");
        telemetry.addData("Both Bumpers", "Intake OFF");
        telemetry.addData("D-Pad Up / Down", "Wrist Up / Down");
        telemetry.addData("Right Stick X", "Intake In / Out");
        telemetry.addData("Right Stick Y", "Slides Up / Down");
        telemetry.addData("Button B", "Delivery Position");
        telemetry.addData("Button A", "Intake Position");
        telemetry.addData("Button X", "Toggle Left Claw");
        telemetry.addData("Button Y", "Toggle Right Claw");
        telemetry.addLine();
        telemetry.addData("Intake Motor Power", robot.intakeMotor.getPower());
        telemetry.addData("Slide Pulley Left Velocity", robot.leftPulley.getVelocity());
        telemetry.addData("Slide Pulley Right Velocity", robot.rightPulley.getVelocity());
        telemetry.addData("Claw Left Position", robot.clawL.getPosition());
        telemetry.addData("Claw Right Position", robot.clawR.getPosition());
        telemetry.addData("Left Passover Position", robot.passoverArmLeft.getPosition());
        telemetry.addData("Right Passover Position", robot.passoverArmRight.getPosition());
        telemetry.addData("Wrist Position", robot.wrist.getPosition());
        telemetry.addData("Left Pulley Position", robot.leftPulley.getPower());
        telemetry.addData("Right Pulley Position", robot.rightPulley.getPower());
        telemetry.addData("> Target Wrist Position", wristPosition);
        telemetry.addData("> Target Passover Position", passoverPosition);
        telemetry.addLine();
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFPower, rightFPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBPower, rightBPower);
        telemetry.addData("Front Left Position", robot.leftFront.getCurrentPosition());
        telemetry.addData("Back Left Position", robot.leftRear.getCurrentPosition());
        telemetry.addData("Front Right Position", robot.rightFront.getCurrentPosition());
        telemetry.addData("Back Right Position", robot.rightRear.getCurrentPosition());

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