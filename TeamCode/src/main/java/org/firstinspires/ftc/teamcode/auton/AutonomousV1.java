package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
        DRIVING_TO_DETECT_INITIAL, DETECTING_INITIAL, DRIVING_TO_DETECT_SECOND, DETECTING_SECOND, DRIVING_TO_LAST_RESORT, DEPOSITING_PURPLE_PIXEL, DRIVING_TO_BACKBOARD, MOVING_AT_BACKBOARD, DEPOSITING_YELLOW_PIXEL, PARK, STOP
    }

    // ------ Element Location Enum ------ //
    private enum ElementLocation {
        LEFT, CENTER, RIGHT, UNKNOWN
    }

    // ------ Alliance Color Enum ------ //
    private enum AllianceColor {
        RED, BLUE
    }

    // ------ Declare Others ------ //
    private HWC robot;
    private final State[] stateSelectionList = new State[]{State.DRIVING_TO_DETECT_INITIAL, State.DRIVING_TO_DETECT_SECOND, State.DEPOSITING_PURPLE_PIXEL, State.DRIVING_TO_BACKBOARD, State.DEPOSITING_YELLOW_PIXEL, State.PARK, State.STOP};
    private State state = State.DRIVING_TO_DETECT_INITIAL;
    private AllianceColor allianceColor = AllianceColor.RED;
    private ElementLocation elementLocation;
    private String activeTrajectory = "";
    private boolean testingMode = false;
    private boolean firstRun = true;
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
    private Trajectory toParkFromBackboardRight;
    private Trajectory toParkFromBackBoardCenter;
    private Trajectory toParkFromBackboardLeft;

    // ------ Starting Position ------ //
    private final Pose2d START_POSE = new Pose2d(12, -60, Math.toRadians(90.00));

    @Override
    public void init() {
        // ------ Telemetry ------ //
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        // ------ Initialize Robot Hardware ------ //
        robot = new HWC(hardwareMap, telemetry, true);

        // ------ Start FTC Dashboard Telemetry ------ //
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // ------ Start Tensorflow ------ //
        robot.initTFOD("fpaVision.tflite");

        // ------ Set Robot Start Pose ------ //
        robot.drive.setPoseEstimate(START_POSE);

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

        // ------ Alliance Color Selection ------ //
        if (robot.currentGamepad1.a && !robot.previousGamepad1.a) {
            allianceColor = allianceColor.equals(AllianceColor.RED) ? AllianceColor.BLUE : AllianceColor.RED;
        }

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
            telemetry.addData("Selected Alliance Color", allianceColor);
            telemetry.addData("Status", "Init Loop");
            telemetry.update();
        }
    }

    @Override
    public void start() {
        // ------ Close Claw for Yellow Pixel ------ //
        robot.toggleClaw('L');

        // ------ Set Trajectories based on Alliance Color ------ //
        switch(allianceColor) {
            case RED:
                // ------ Declare Trajectories ------ //
                // Driving to Initial Detection Location
                toDetectInitial = robot.drive.trajectoryBuilder(START_POSE)
                        .lineTo(new Vector2d(12.0, -42))
                        .build();

                // Driving to Second Detection Location
                toDetectSecond = robot.drive.trajectoryBuilder(toDetectInitial.end())
                        .strafeTo(new Vector2d(23, -42))
                        .build();

                // Driving to Last Resort
                toLastResort = robot.drive.trajectoryBuilder(toDetectSecond.end())
                        .lineToLinearHeading(new Pose2d(7, -38, Math.toRadians(135)))
                        .build();

                // Driving to Backboard from Initial Detection Location
                toBackboardFromInitial = robot.drive.trajectoryBuilder(toDetectInitial.end())
                        .lineToLinearHeading(new Pose2d(43, -35, Math.toRadians(180)))
                        .build();

                // Driving to Backboard from Second Detection Location
                toBackboardFromSecond = robot.drive.trajectoryBuilder(toDetectSecond.end())
                        .lineToLinearHeading(new Pose2d(43, -35, Math.toRadians(180)))
                        .build();

                // Driving to Backboard from Last Resort
                toBackboardFromLastResort = robot.drive.trajectoryBuilder(toLastResort.end())
                        .lineToLinearHeading(new Pose2d(43, -35, Math.toRadians(180)))
                        .build();

                // Left Side Score
                toBackboardLeft = robot.drive.trajectoryBuilder(new Pose2d(50.87, -36.89, Math.toRadians(9.69)))
                        .strafeLeft(10) // TODO: Review Distance
                        .build();

                // Right Side Score
                toBackboardRight = robot.drive.trajectoryBuilder(new Pose2d(50.87, -36.89, Math.toRadians(9.69)))
                        .strafeRight(10) // TODO: Review Distance
                        .build();

                // Park from Backboard Right
                toParkFromBackboardRight = robot.drive.trajectoryBuilder(toBackboardRight.end())
                        .strafeRight(10) // TODO: Review Distance
                        .build();

                // Park from Backboard Center
                toParkFromBackBoardCenter = robot.drive.trajectoryBuilder(toBackboardFromInitial.end())
                        .strafeRight(15) // TODO: Review Distance
                        .build();

                // Park from Backboard Left
                toParkFromBackboardLeft = robot.drive.trajectoryBuilder(toBackboardLeft.end())
                        .strafeRight(20) // TODO: Review Distance
                        .build();

                break;
            case BLUE:
                // ------ Declare Trajectories ------ //
                // Driving to Initial Detection Location
                toDetectInitial = robot.drive.trajectoryBuilder(START_POSE)
                        .lineTo(new Vector2d(12.0, 42))
                        .build();

                // Driving to Second Detection Location
                toDetectSecond = robot.drive.trajectoryBuilder(toDetectInitial.end())
                        .strafeTo(new Vector2d(23, 42))
                        .build();

                // Driving to Last Resort
                toLastResort = robot.drive.trajectoryBuilder(toDetectSecond.end())
                        .lineToLinearHeading(new Pose2d(7, 38, Math.toRadians(225)))
                        .build();

                // Driving to Backboard from Initial Detection Location
                toBackboardFromInitial = robot.drive.trajectoryBuilder(toDetectInitial.end())
                        .lineToLinearHeading(new Pose2d(43, 35, Math.toRadians(180)))
                        .build();

                // Driving to Backboard from Second Detection Location
                toBackboardFromSecond = robot.drive.trajectoryBuilder(toDetectSecond.end())
                        .lineToLinearHeading(new Pose2d(43, 35, Math.toRadians(180)))
                        .build();

                // Driving to Backboard from Last Resort
                toBackboardFromLastResort = robot.drive.trajectoryBuilder(toLastResort.end())
                        .lineToLinearHeading(new Pose2d(43, 35, Math.toRadians(180)))
                        .build();

                // Left Side Score
                toBackboardLeft = robot.drive.trajectoryBuilder(new Pose2d(50.87, -36.89, Math.toRadians(9.69)))
                        .strafeLeft(10) // TODO: Review Distance
                        .build();

                // Right Side Score
                toBackboardRight = robot.drive.trajectoryBuilder(new Pose2d(50.87, -36.89, Math.toRadians(9.69)))
                        .strafeRight(10) // TODO: Review Distance
                        .build();

                // Park from Backboard Right
                toParkFromBackboardRight = robot.drive.trajectoryBuilder(toBackboardRight.end())
                        .strafeLeft(10) // TODO: Review Distance
                        .build();

                // Park from Backboard Center
                toParkFromBackBoardCenter = robot.drive.trajectoryBuilder(toBackboardFromInitial.end())
                        .strafeLeft(15) // TODO: Review Distance
                        .build();

                // Park from Backboard Left
                toParkFromBackboardLeft = robot.drive.trajectoryBuilder(toBackboardLeft.end())
                        .strafeLeft(20) // TODO: Review Distance
                        .build();

                break;
        }
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
            case PARK:
                drivingToPark();
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
        if(firstRun) {
            robot.drive.followTrajectoryAsync(toDetectInitial);
            firstRun = false;
        }

        // ------ Set Next State ------ //
        if (!robot.drive.isBusy()) {
            state = State.DETECTING_INITIAL;
            firstRun = true;
        }
    }

    // Detect Initial Location
    private void detectingInitial() {
        // ------ Reset WaitTime First Run ------ //
        if(firstRun) {
            robot.time.reset();
            firstRun = false;
        }

        // ------ Detect Element ------ //
        if (robot.time.seconds() <= 3) {
            if (robot.detectElement()) {
                elementLocation = ElementLocation.CENTER;
            } else {
                elementLocation = ElementLocation.UNKNOWN;
            }
        }

        // ------ Set Next State ------ //
        if (elementLocation == ElementLocation.CENTER) {
            state = State.DRIVING_TO_BACKBOARD;
            firstRun = true;
        } else {
            state = State.DRIVING_TO_DETECT_SECOND;
            firstRun = true;
        }
    }

    // Drive to Second Detection Location
    private void drivingToDetectSecond() {
        // ------ Set Active Trajectory ------ //
        activeTrajectory = "toDetectSecond";

        // ------ Follow Trajectory ------ //
        if(firstRun) {
            robot.drive.followTrajectoryAsync(toDetectSecond);
            firstRun = false;
        }

        // ------ Set Next State ------ //
        if (!robot.drive.isBusy()) {
            state = State.DETECTING_SECOND;
            firstRun = true;
        }
    }

    // Detect Second Location
    private void detectingSecond() {
        // ------ Reset WaitTime First Run ------ //
        if(firstRun) {
            robot.time.reset();
            firstRun = false;
        }

        // ------ Detect Element ------ //
        if (robot.time.seconds() <= 3) {
            if (robot.detectElement()) {
                elementLocation = ElementLocation.RIGHT;
            } else {
                elementLocation = ElementLocation.LEFT;
            }
        }

        // ------ Set Next State ------ //
        if (elementLocation == ElementLocation.RIGHT) {
            state = State.DRIVING_TO_BACKBOARD;
            firstRun = true;
        } else {
            state = State.DRIVING_TO_LAST_RESORT;
            firstRun = true;
        }
    }

    // Drive to Last Resort
    private void drivingToLastResort() {
        // ------ Set Active Trajectory ------ //
        activeTrajectory = "toLastResort";

        // ------ Follow Trajectory ------ //
        if(firstRun) {
            robot.drive.followTrajectoryAsync(toLastResort);
            firstRun = false;
        }

        // ------ Set Next State ------ //
        if (!robot.drive.isBusy()) {
            state = State.DEPOSITING_PURPLE_PIXEL;
            firstRun = true;
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
        if(firstRun) {
            firstRun = false;
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
        }

        // ------ Set Next State ------ //
        if (!robot.drive.isBusy()) {
            state = State.MOVING_AT_BACKBOARD;
            firstRun = true;
        }
    }

    // Move at Backboard
    private void movingAtBackboard() {
        // ------ Select Trajectory ------ //
        if(firstRun) {
            firstRun = false;
            if (elementLocation == ElementLocation.RIGHT) {
                activeTrajectory = "toBackboardRight";
                robot.drive.followTrajectoryAsync(toBackboardRight);
            } else if (elementLocation == ElementLocation.LEFT) {
                activeTrajectory = "toBackboardLeft";
                robot.drive.followTrajectoryAsync(toBackboardLeft);
            }
        }

        // ------ Set Next State ------ //
        if (!robot.drive.isBusy()) {
            state = State.DEPOSITING_YELLOW_PIXEL;
            firstRun = true;
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
        state = State.PARK;
    }

    // Drive to Park
    private void drivingToPark() {
        // ------ Select Trajectory ------ //
        if(firstRun) {
            firstRun = false;
            if (elementLocation == ElementLocation.RIGHT) {
                activeTrajectory = "toParkFromBackboardRight";
                robot.drive.followTrajectoryAsync(toParkFromBackboardRight);
            } else if (elementLocation == ElementLocation.LEFT) {
                activeTrajectory = "toParkFromBackboardLeft";
                robot.drive.followTrajectoryAsync(toParkFromBackboardLeft);
            } else {
                activeTrajectory = "toParkFromBackboardCenter";
                robot.drive.followTrajectoryAsync(toParkFromBackBoardCenter);
            }
        }

        // ------ Set Next State ------ //
        if (!robot.drive.isBusy()) {
            state = State.STOP;
            firstRun = true;
        }
    }
}