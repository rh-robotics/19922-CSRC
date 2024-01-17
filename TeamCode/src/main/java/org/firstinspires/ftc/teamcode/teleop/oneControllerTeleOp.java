package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.RobotState;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.HWC;

/**
 * Enum representing which speed to change in init_loop()
 */
enum Selection {
    TURN_SPEED,
    DRIVE_SPEED,
    STRAFE_SPEED
}

/**
 * TeleOp OpMode for simply driving with strafing wheels
 */
@TeleOp(name = "1P TeleOp", group = "Iterative OpMode")
public class oneControllerTeleOp extends OpMode {
    HWC robot; // Declare the object for HWC, will allow us to access all the motors declared there!
    RobotState state = RobotState.RESTING;
    double turnSpeed = 0.5; // Speed multiplier for turning
    double driveSpeed = 1; // Speed multiplier for driving
    double strafeSpeed = 0.8; // Speed multiplier for strafing
    double armPwr = 0;
    Selection selection = Selection.TURN_SPEED; // String for selecting which speed to change

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
        // Reset Servos to Position 0
        robot.clawL.setPosition(1);
        robot.clawR.setPosition(0);

        // Select which speed to change
        if (gamepad1.a) {
            selection = Selection.TURN_SPEED;
        } else if (gamepad1.b) {
            selection = Selection.DRIVE_SPEED;
        } else if (gamepad1.x) {
            selection = Selection.STRAFE_SPEED;
        }

        // Change the speed multiplier for selection
        switch (selection) {
            case TURN_SPEED:
                if (gamepad1.dpad_up) {
                    turnSpeed += 0.1;
                } else if (gamepad1.dpad_down) {
                    turnSpeed -= 0.1;
                }
                break;
            case DRIVE_SPEED:
                if (gamepad1.dpad_up) {
                    driveSpeed += 0.1;
                } else if (gamepad1.dpad_down) {
                    driveSpeed -= 0.1;
                }
                break;
            case STRAFE_SPEED:
                if (gamepad1.dpad_up) {
                    strafeSpeed += 0.1;
                } else if (gamepad1.dpad_down) {
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
        double leftFPower;
        double rightFPower;
        double leftBPower;
        double rightBPower;
        double drive = -gamepad1.left_stick_x * driveSpeed;
        double turn = (gamepad1.right_trigger-gamepad1.left_trigger) * turnSpeed;
        double strafe = -gamepad1.left_stick_x * strafeSpeed;

        // --------------- Calculate drive power --------------- //
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
        if (gamepad1.right_stick_x != 0) {
            robot.oldIntake(gamepad1.right_stick_x);
            state = RobotState.INTAKING;
        }

        // ------ Pulley ------ //
        if (gamepad1.right_stick_y != 0) {
            robot.manualArm(-gamepad1.right_stick_y);
        } else {
            robot.manualArm(0);
        }

        // ------ Claw Controls ------ //
        if (gamepad1.x) {
            robot.toggleClaw('L');
        }
        if (gamepad1.y) {
            robot.toggleClaw('R');
        }



        // --------------- Run Drive Motors --------------- //
        robot.leftFront.setPower(leftFPower);
        robot.rightFront.setPower(rightFPower);
        robot.leftRear.setPower(leftBPower);
        robot.rightRear.setPower(rightBPower);

        if (gamepad1.left_bumper){
            armPwr = 0.8;
        }
        else if (gamepad1.right_bumper){
            armPwr = -0.8;
        }
        robot.passoverArmLeft.setPower(armPwr);
        robot.passoverArmRight.setPower(-armPwr);

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
        if (gamepad1.y){
            robot.moveWrist(1,-1);
        }
        else if (gamepad1.a){
            robot.moveWrist(0,0);
        }

        // --------------- Telemetry Updates --------------- //
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