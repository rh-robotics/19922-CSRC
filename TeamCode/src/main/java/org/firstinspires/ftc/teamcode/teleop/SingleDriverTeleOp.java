package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.HWC;
import org.firstinspires.ftc.teamcode.teleop.enums.MultiplierSelection;

/**
 * TeleOp OpMode for simply driving with strafing wheels
 */
@TeleOp(name = "Single Driver", group = "Primary OpModes")
public class SingleDriverTeleOp extends OpMode {
    // ------ Intake State Enum ------ //
    private enum IntakeState {INTAKE, OFF, OUTTAKE}

    // ------ Endgame State Enum ------ //
    private enum EndgameState {DRONE, SLIDES_UP, CLIMB}

    // ------ Aligning Boolean ------ //
    private enum AligningState {ALIGNING, DRIVING}

    // ------ Declare Others ------ //
    private HWC robot;
    private MultiplierSelection selection = MultiplierSelection.TURN_SPEED;
    private IntakeState intakeState = IntakeState.OFF;
    private EndgameState endgameState = EndgameState.DRONE;
    private AligningState aligningState = AligningState.DRIVING;
    private boolean testingMode = false;
    private double turnSpeed = 0.5; // Speed multiplier for turning
    private double driveSpeed = 1; // Speed multiplier for driving
    private double strafeSpeed = 0.8; // Speed multiplier for strafing

    // ------ Other Variables ------ //
    private int slideHeight = 0;
    private double passoverPosition = HWC.passoverIntakePos;
    private double wristPosition = HWC.wristIntakePos;
    private double frontLeftPower;
    private double frontRightPower;
    private double backLeftPower;
    private double backRightPower;

    @Override
    public void init() {
        // ------ Telemetry ------ //
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        // ------ Initialize Robot Hardware ------ //
        robot = new HWC(hardwareMap, telemetry, false);

        // ------ Reset Pulley Encoders ------ //
        robot.leftPulley.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightPulley.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // ------ Reset Servos ------ //
        robot.clawL.setPosition(1);
        robot.clawR.setPosition(0);
        robot.drone.setPosition(1);
        robot.wrist.setPosition(HWC.wristIntakePos);
        robot.passoverArmLeft.setPosition(HWC.passoverIntakePos);
        robot.passoverArmRight.setPosition(HWC.passoverIntakePos);

        // ------ Reset Pulley Encoder Positions ------ //
        resetSlideEncoders();

        // ------ Telemetry ------ //
        telemetry.addData("> Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // ------ GamePad Updates ------ //
        robot.previousGamepad1.copy(robot.currentGamepad1);
        robot.currentGamepad1.copy(gamepad1);

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
        if ((robot.currentGamepad1.left_bumper && !robot.previousGamepad1.left_bumper) && (robot.currentGamepad1.right_bumper && !robot.previousGamepad1.right_bumper)) {
            testingMode = !testingMode;
        }

        // ------ Telemetry ------ //
        if (testingMode) {
            telemetry.addData("> TESTING MODE IS ENABLED", "Extra Telemetry is Displayed");
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
        robot.currentGamepad1.copy(gamepad1);

        // ------ Power & Controller Values ------ //
        double drive = -robot.currentGamepad1.left_stick_y;
        double strafe = robot.currentGamepad1.left_stick_x;
        double turn = (robot.currentGamepad1.left_trigger - robot.currentGamepad1.right_trigger) * turnSpeed;

        double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1);
        frontLeftPower = (turn - strafe - drive) / denominator;
        backLeftPower = (turn + strafe - drive) / denominator;
        frontRightPower = (turn - strafe + drive) / denominator;
        backRightPower = (turn + strafe + drive) / denominator;

        passoverPosition = robot.passoverArmLeft.getPosition();
        wristPosition = robot.wrist.getPosition();

        // ------ Claw Controls ------ //
        if (robot.currentGamepad1.x && !robot.previousGamepad1.x) {
            robot.toggleClaw('L');
        }
        if (robot.currentGamepad1.y && !robot.previousGamepad1.y) {
            robot.toggleClaw('R');
        }

        // ------ Intake Toggle Controls ------ //
        if (robot.currentGamepad1.right_bumper && !robot.previousGamepad1.right_bumper) {
            if (intakeState == IntakeState.INTAKE || intakeState == IntakeState.OUTTAKE) {
                intakeState = IntakeState.OFF;
            } else if (intakeState == IntakeState.OFF) {
                intakeState = IntakeState.INTAKE;
            }
        }
        if (robot.currentGamepad1.left_bumper && !robot.previousGamepad1.left_bumper) {
            if (intakeState == IntakeState.INTAKE || intakeState == IntakeState.OUTTAKE) {
                intakeState = IntakeState.OFF;
            } else if (intakeState == IntakeState.OFF) {
                intakeState = IntakeState.OUTTAKE;
            }
        }

        // ------ Slide Position Controls ------ //
        if (robot.currentGamepad1.dpad_up && !robot.previousGamepad1.dpad_up) {
            // Increment Value
            slideHeight++;

            // If value is above the max, don't increase
            if (slideHeight > HWC.slidePositions.length - 1) {
                slideHeight = HWC.slidePositions.length - 1;
            }
        }
        if (robot.currentGamepad1.dpad_down && !robot.previousGamepad1.dpad_down) {
            // Increment Value
            slideHeight--;

            // If value is below 0, don't decrease
            if (slideHeight < 0) {
                slideHeight = 0;
            }
        }

        // ------ Reset Pulley Encoders ------ //
        if (robot.currentGamepad1.right_stick_button && !robot.previousGamepad1.right_stick_button) {
            resetSlideEncoders();
        }

        // ------ Start Alignment with Backboard ------ //
        if (robot.currentGamepad1.left_stick_button && !robot.previousGamepad1.left_stick_button) {
            aligningState = AligningState.ALIGNING;
        }

        // ------ Passover & Claw Position Controls ------ //
        if (robot.currentGamepad1.b && !robot.previousGamepad1.b) {
            deliveryPosition();
        } else if (robot.currentGamepad1.a && !robot.previousGamepad1.a) {
            intakePosition();
        }

        // ------ Endgame Controls ------ //
        if (robot.currentGamepad1.back && !robot.previousGamepad1.back) {
            switch (endgameState) {
                case DRONE:
                    // Launch Drone
                    robot.drone.setPosition(HWC.droneLaunchPos);

                    // Set Next State
                    endgameState = EndgameState.SLIDES_UP;

                    break;
                case SLIDES_UP:
                    // Set Slide Height
                    slideHeight = 5;

                    // Set Next State
                    endgameState = EndgameState.CLIMB;

                    break;
                case CLIMB:
                    // Set Slide Height
                    slideHeight = 0;

                    break;
            }
        }

        // -------- Check Intake State & Run Intake ------ //
        switch (intakeState) {
            case INTAKE:
                // Set Power
                robot.intakeMotor.setPower(-1);

                // Check Intake
                detectIntake();

                break;
            case OFF:
                // Give Manual Control to Gamepad 2
                robot.intakeMotor.setPower(robot.currentGamepad2.right_stick_x);

                break;
            case OUTTAKE:
                // Set Power
                robot.intakeMotor.setPower(1);

                break;
        }

        // ------ Alignment / Driving ------ //
        switch (aligningState) {
            case DRIVING:
                break;
            case ALIGNING:
                alignWithBackboard();
                break;
        }

        // ------ Set Slide Positions ------ //
        robot.pulleyLComponent.setTarget(HWC.slidePositions[slideHeight]);
        robot.pulleyRComponent.setTarget(HWC.slidePositions[slideHeight]);

        // ------ Run Motors ------ //
        robot.leftFront.setPower(frontLeftPower);
        robot.leftRear.setPower(backLeftPower);
        robot.rightFront.setPower(frontRightPower);
        robot.rightRear.setPower(backRightPower);

        // ------ Run Servos ------ //
        robot.passoverArmLeft.setPosition(passoverPosition);
        robot.passoverArmRight.setPosition(passoverPosition);
        robot.wrist.setPosition(wristPosition);

        // ------ Run Slides Using PID ------ //
        robot.pulleyLComponent.moveUsingPID();
        robot.pulleyRComponent.moveUsingPID();

        // ------ Telemetry Updates ------ //
        telemetry.addData("> Status", "Running");
        telemetry.addData("Intake State", intakeState);
        telemetry.addData("Endgame State", endgameState);
        telemetry.addData("Aligning State", aligningState);
        telemetry.addData("Slide Height", slideHeight);

        // ------ Testing Mode Telemetry ------ //
        if (testingMode) {
            telemetry.addLine();
            telemetry.addData("> Testing Mode", "Enabled");
            telemetry.addData("Left Distance", robot.distLeft.getDistance(DistanceUnit.CM));
            telemetry.addData("Right Distance", robot.distRight.getDistance(DistanceUnit.CM));
            telemetry.addData("Passover Position", passoverPosition);
            telemetry.addData("Wrist Position", wristPosition);
            telemetry.addData("Slide L Position", robot.leftPulley.getCurrentPosition());
            telemetry.addData("Slide R Position", robot.rightPulley.getCurrentPosition());
            telemetry.addData("Slide L Target", robot.pulleyLComponent.getTarget());
            telemetry.addData("Slide R Target", robot.pulleyRComponent.getTarget());
            telemetry.addData("Front Left Power", frontLeftPower);
            telemetry.addData("Front Right Power", frontRightPower);
            telemetry.addData("Back Left Power", backLeftPower);
            telemetry.addData("Back Right Power", backRightPower);
            telemetry.addData("Color Left Distance", robot.colorLeft.getDistance(DistanceUnit.CM));
            telemetry.addData("Color Right Distance", robot.colorRight.getDistance(DistanceUnit.CM));

        }
        telemetry.update();
    }

    private void deliveryPosition() {
        wristPosition = HWC.wristDeliveryPos;
        passoverPosition = HWC.passoverDeliveryPos;

        if (slideHeight == 0) {
            slideHeight = 1;
        }
    }

    private void intakePosition() {
        wristPosition = HWC.wristIntakePos;
        passoverPosition = HWC.passoverIntakePos;

        robot.clawL.setPosition(1);
        robot.clawR.setPosition(0);

        slideHeight = 0;
    }

    private void resetSlideEncoders() {
        // Reset Encoders
        robot.leftPulley.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightPulley.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Run Without Encoders
        robot.leftPulley.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightPulley.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void alignWithBackboard() {
        int distPlus = 17 + 2;
        int distMinus = 17 - 2;

        if (distPlus >= robot.distRight.getDistance(DistanceUnit.CM) && robot.distRight.getDistance(DistanceUnit.CM) >= distMinus) {
            frontRightPower = 0;
            backRightPower = 0;

            if (distPlus >= robot.distLeft.getDistance(DistanceUnit.CM) && robot.distLeft.getDistance(DistanceUnit.CM) >= distMinus) {
                frontLeftPower = 0;
                backLeftPower = 0;
                aligningState = AligningState.DRIVING;
            } else if (robot.distRight.getDistance(DistanceUnit.CM) < distMinus) {
                frontLeftPower = 0.3;
                backLeftPower = 0.3;
            }
        } else if (distPlus >= robot.distLeft.getDistance(DistanceUnit.CM) && robot.distLeft.getDistance(DistanceUnit.CM) >= distMinus) {
            frontLeftPower = 0;
            backLeftPower = 0;

            if (distPlus >= robot.distRight.getDistance(DistanceUnit.CM) && robot.distRight.getDistance(DistanceUnit.CM) >= distMinus) {
                frontRightPower = 0;
                backRightPower = 0;
                aligningState = AligningState.DRIVING;
            } else if (robot.distRight.getDistance(DistanceUnit.CM) > distPlus) {
                frontRightPower = -0.3;
                backRightPower = -0.3;
            }
        } else {
            frontLeftPower = 0.2;
            frontRightPower = -0.2;
            backLeftPower = 0.2;
            backRightPower = -0.2;
        }

        if (distPlus >= robot.distRight.getDistance(DistanceUnit.CM) && robot.distRight.getDistance(DistanceUnit.CM) >= distMinus && distPlus >= robot.distLeft.getDistance(DistanceUnit.CM) && robot.distLeft.getDistance(DistanceUnit.CM) >= distMinus) {
            frontLeftPower = 0;
            backLeftPower = 0;
            frontRightPower = 0;
            backRightPower = 0;
            aligningState = AligningState.DRIVING;
        } else {
            if (distMinus > robot.distRight.getDistance(DistanceUnit.CM) && distMinus > robot.distLeft.getDistance(DistanceUnit.CM)) {
                frontLeftPower = -0.2;
                backLeftPower = -0.2;
                frontRightPower = 0.2;
                backRightPower = 0.2;
            } else if (distMinus > robot.distRight.getDistance(DistanceUnit.CM)) {
                frontLeftPower = 0;
                backLeftPower = 0;
                frontRightPower = 0.3;
                backRightPower = 0.3;
            } else if (distMinus > robot.distLeft.getDistance(DistanceUnit.CM)) {
                frontLeftPower = -0.3;
                backLeftPower = -0.3;
                frontRightPower = 0;
                backRightPower = 0;
            }

        }
    }

    private void detectIntake() {
        // Close Claws when Pixel Detected
        if (robot.colorLeft.getDistance(DistanceUnit.CM) <= 2) {
            robot.clawL.setPosition(0.5);
        }

        if (robot.colorRight.getDistance(DistanceUnit.CM) <= 2) {
            robot.clawR.setPosition(0.5);
        }

        // If both Pixels are Detected, Stop Intake
        if (robot.colorLeft.getDistance(DistanceUnit.CM) <= 1.5 && robot.colorRight.getDistance(DistanceUnit.CM) <= 1.5) {
            intakeState = IntakeState.OFF;
        }
    }
}