package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.HWC;

@Autonomous(name = "AutonomousV1")
public class AutonomousV1 extends OpMode {
    // ------ State Enum ------ //
    private enum State {
        DRIVING_TO_DETECT_INITIAL, DETECTING_INITIAL, DRIVING_TO_DETECT_SECOND, DETECTING_SECOND, DRIVING_TO_LAST_RESORT, DEPOSITING_PURPLE_PIXEL, DRIVING_TO_BACKBOARD, MOVING_AT_BACKBOARD, DEPOSITING_YELLOW_PIXEL, STOP
    }

    // ------ Element Location Enum ------ //
    private enum ElementLocation {
        LEFT, CENTER, RIGHT, UNKNOWN
    }

    // ------ Declare Others ------ //
    private HWC robot;
    private State[] stateSelectionList = new State[]{State.DRIVING_TO_DETECT_INITIAL, State.DRIVING_TO_DETECT_SECOND, State.DEPOSITING_PURPLE_PIXEL, State.DRIVING_TO_BACKBOARD, State.DEPOSITING_YELLOW_PIXEL, State.STOP};

    private State state = State.DRIVING_TO_DETECT_INITIAL;
    private ElementLocation elementLocation;
    private String activeTrajectory = "";
    private boolean testingMode = false;
    private int stateIndex = 0;

    // ------ Trajectories ------ //
    private Trajectory toDetectInitial;
    private Trajectory toDetectSecond;
    private Trajectory toLastResort;
    private Trajectory toBackboardFromInitial;
    private Trajectory toBackboardFromSecond;
    private Trajectory toBackboardFromLastResort;
    private Trajectory toBackboardLeft;
    private Trajectory toBackboardRight;

    // ------ Starting Position ------ //
    private Pose2d startPose = new Pose2d(11.84, -60.75, Math.toRadians(90.00));

    @Override
    public void init() {
        // ------ Telemetry ------ //
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        // ------ Initialize Robot Hardware ------ //
        robot = new HWC(hardwareMap, telemetry, true);

        // ------ Start Tensorflow ------ //
        robot.initTFOD("fpaVision.tflite");

        // ------ Set Robot Start Pose ------ //
        robot.drive.setPoseEstimate(startPose);

        // ------ Declare Trajectories ------ //
        // Driving to Initial Detection Location
        toDetectInitial = robot.drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(11.67, -44.39))
                .build();

        // Driving to Second Detection Location
        toDetectSecond = robot.drive.trajectoryBuilder(toDetectInitial.end())
                .lineTo(new Vector2d(23.77, -43.88))
                .build();

        // Driving to Last Resort
        toLastResort = robot.drive.trajectoryBuilder(toDetectSecond.end())
                .lineTo(new Vector2d(11.67, -44.39))
                .build();

        // Driving to Backboard from Initial Detection Location
        toBackboardFromInitial = robot.drive.trajectoryBuilder(toDetectInitial.end())
                .splineTo(new Vector2d(50.87, -36.89), Math.toRadians(9.69))
                .build();

        // Driving to Backboard from Second Detection Location
        toBackboardFromSecond = robot.drive.trajectoryBuilder(toDetectSecond.end())
                .splineTo(new Vector2d(50.87, -36.89), Math.toRadians(9.69))
                .build();

        // Driving to Backboard from Last Resort
        toBackboardFromLastResort = robot.drive.trajectoryBuilder(toLastResort.end())
                .splineTo(new Vector2d(50.87, -36.89), Math.toRadians(9.69))
                .build();

        // Left Side Score
        toBackboardLeft = robot.drive.trajectoryBuilder(new Pose2d(50.87, -36.89, Math.toRadians(9.69)))
                .strafeLeft(10) // TODO: Review Distance
                .build();

        // Right Side Score
        toBackboardRight = robot.drive.trajectoryBuilder(new Pose2d(50.87, -36.89, Math.toRadians(9.69)))
                .strafeRight(10) // TODO: Review Distance
                .build();

        // ------ Reset Servos ------ //
        robot.clawL.setPosition(1);
        robot.clawR.setPosition(0);

        // ------ Telemetry ------ //
        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "Yellow Pixel on Left Side");
        telemetry.addData(">", "Purple Pixel in intake");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // ------ GamePad Updates ------ //
        robot.previousGamepad1.copy(robot.currentGamepad1);
        robot.previousGamepad2.copy(robot.currentGamepad2);
        robot.currentGamepad1.copy(gamepad1);
        robot.currentGamepad2.copy(gamepad2);

        // ------ Enabling Testing Mode ------ //
        if ((robot.currentGamepad1.right_bumper && !robot.previousGamepad1.right_bumper) && (robot.currentGamepad1.right_bumper && !robot.previousGamepad1.right_bumper)) {
            testingMode = !testingMode;
        }

        if (testingMode) {
            if (robot.currentGamepad1.dpad_up && !robot.previousGamepad1.dpad_up) {
                stateIndex++;
                if (stateIndex >= stateSelectionList.length) {
                    stateIndex = 0;
                }
            } else if (robot.currentGamepad1.dpad_down && !robot.previousGamepad1.dpad_down) {
                stateIndex--;
                if (stateIndex < 0) {
                    stateIndex = stateSelectionList.length - 1;
                }

            }
            state = stateSelectionList[stateIndex];
        }

        // ------ Telemetry ------ //
        if(testingMode) {
            telemetry.addData(">", "TESTING MODE ENABLED");
            telemetry.addData("Selected State", state);
            telemetry.addLine("-----------------------------");
        } else {
            telemetry.addData(">", "Yellow Pixel on Left Side");
            telemetry.addData(">", "Purple Pixel in intake");
            telemetry.addData("Status", "Init Loop");
            telemetry.update();
        }
    }

    @Override
    public void start() {
        // ------ Close Claw for Yellow Pixel ------ //
        robot.toggleClaw('L');
    }

    @Override
    public void loop() {
        // ------ GamePad Updates ------ //
        robot.previousGamepad1.copy(robot.currentGamepad1);
        robot.previousGamepad2.copy(robot.currentGamepad2);
        robot.currentGamepad1.copy(gamepad1);
        robot.currentGamepad2.copy(gamepad2);

        // ------ State Machine ------ //
        switch (state) {
            case DRIVING_TO_DETECT_INITIAL:
                drivingToDetectInitial();
                break;
            case DETECTING_INITIAL:
                detectingInitial();
                break;
            case DRIVING_TO_DETECT_SECOND:
                drivingToDetectSecond();
                break;
            case DETECTING_SECOND:
                detectingSecond();
                break;
            case DRIVING_TO_LAST_RESORT:
                drivingToLastResort();
                break;
            case DEPOSITING_PURPLE_PIXEL:
                depositingPurplePixel();
                break;
            case DRIVING_TO_BACKBOARD:
                drivingToBackboard();
                break;
            case MOVING_AT_BACKBOARD:
                movingAtBackboard();
                break;
            case DEPOSITING_YELLOW_PIXEL:
                depositingYellowPixel();
                break;
            case STOP:
                stop();
                break;
        }

        // ------ Move Robot ------ //
        robot.drive.update();

        // ------ Telemetry ------ //
        telemetry.addData("Status", state);
        telemetry.addData("Element Location", elementLocation);
        telemetry.addData("Pose", robot.drive.getPoseEstimate());
        telemetry.addData("Active Trajectory", activeTrajectory);
        telemetry.addLine();
        telemetry.addData("Hardware", robot);
        telemetry.update();
    }

    // ------ State Methods ------ //
    // Drive to Initial Detection Location
    private void drivingToDetectInitial() {
        // ------ Set Active Trajectory ------ //
        activeTrajectory = "toDetectInitial";

        // ------ Follow Trajectory ------ //
        robot.drive.followTrajectoryAsync(toDetectInitial);

        // ------ Set Next State ------ //
        if (!robot.drive.isBusy()) {
            state = State.DETECTING_INITIAL;
        }
    }

    // Detect Initial Location
    private void detectingInitial() {
        // ------ Detect Element ------ //
        // TODO: This will probably cause issues. We likely need to wait a bit while detecting
        if (robot.detectElement()) {
            elementLocation = ElementLocation.CENTER;
        } else {
            elementLocation = ElementLocation.UNKNOWN;
        }

        // ------ Set Next State ------ //
        if (elementLocation == ElementLocation.CENTER) {
            state = State.DRIVING_TO_BACKBOARD;
        } else {
            state = State.DRIVING_TO_DETECT_SECOND;
        }
    }

    // Drive to Second Detection Location
    private void drivingToDetectSecond() {
        // ------ Set Active Trajectory ------ //
        activeTrajectory = "toDetectSecond";

        // ------ Follow Trajectory ------ //
        robot.drive.followTrajectoryAsync(toDetectSecond);

        // ------ Set Next State ------ //
        if (!robot.drive.isBusy()) {
            state = State.DETECTING_SECOND;
        }
    }

    // Detect Second Location
    private void detectingSecond() {
        // ------ Detect Element ------ //
        // TODO: This will probably cause issues. We likely need to wait a bit while detecting
        if (robot.detectElement()) {
            elementLocation = ElementLocation.RIGHT;
        } else {
            elementLocation = ElementLocation.LEFT;
        }

        // ------ Set Next State ------ //
        if (elementLocation == ElementLocation.RIGHT) {
            state = State.DRIVING_TO_BACKBOARD;
        } else {
            state = State.DRIVING_TO_LAST_RESORT;
        }
    }

    // Drive to Last Resort
    private void drivingToLastResort() {
        // ------ Set Active Trajectory ------ //
        activeTrajectory = "toLastResort";

        // ------ Follow Trajectory ------ //
        robot.drive.followTrajectoryAsync(toLastResort);

        // ------ Set Next State ------ //
        if (!robot.drive.isBusy()) {
            state = State.DEPOSITING_PURPLE_PIXEL;
        }
    }

    // Deposit Purple Pixel
    private void depositingPurplePixel() {
        // ------ Run Intake Motor Backwards for 1.5 Seconds ------ //
        robot.intakeMotor.setPower(-0.5);
        robot.elapsedTimeSleep(1500);
        robot.intakeMotor.setPower(0);

        // ------ Set Next State ------ //
        state = State.DRIVING_TO_BACKBOARD;
    }

    // Drive to Backboard
    private void drivingToBackboard() {
        // ------ Select Trajectory ------ //
        if (elementLocation == ElementLocation.CENTER) {
            activeTrajectory = "toBackboardFromInitial";
            robot.drive.followTrajectoryAsync(toBackboardFromInitial);
        } else if (elementLocation == ElementLocation.RIGHT) {
            activeTrajectory = "toBackboardFromSecond";
            robot.drive.followTrajectoryAsync(toBackboardFromSecond);
        } else {
            activeTrajectory = "toBackboardFromLastResort";
            robot.drive.followTrajectoryAsync(toBackboardFromLastResort);
        }

        // ------ Set Next State ------ //
        if (!robot.drive.isBusy()) {
            state = State.MOVING_AT_BACKBOARD;
        }
    }

    // Move at Backboard
    private void movingAtBackboard() {
        // ------ Select Trajectory ------ //
        if (elementLocation == ElementLocation.RIGHT) {
            activeTrajectory = "toBackboardRight";
            robot.drive.followTrajectoryAsync(toBackboardRight);
        } else if (elementLocation == ElementLocation.LEFT) {
            activeTrajectory = "toBackboardLeft";
            robot.drive.followTrajectoryAsync(toBackboardLeft);
        }

        // ------ Set Next State ------ //
        if (!robot.drive.isBusy()) {
            state = State.DEPOSITING_YELLOW_PIXEL;
        }
    }

    // Deposit Yellow Pixel
    private void depositingYellowPixel() {
        // ------ Move Passover & Wrist ------ //
        robot.passoverArmLeft.setPosition(HWC.passoverDeliveryPos);
        robot.wrist.setPosition(HWC.wristDeliveryPos);

        // ------ Wait for Passover to Move ------ //
        robot.elapsedTimeSleep(500);

        // ------ Open Claw ------ //
        robot.toggleClaw('L');

        // ------ Wait for Claw to Open ------ //
        robot.elapsedTimeSleep(500);

        // ------ Move Passover & Wrist ------ //
        robot.passoverArmLeft.setPosition(HWC.passoverIntakePos);
        robot.wrist.setPosition(HWC.wristIntakePos);

        // ------ Set Next State ------ //
        state = State.STOP;
    }
}