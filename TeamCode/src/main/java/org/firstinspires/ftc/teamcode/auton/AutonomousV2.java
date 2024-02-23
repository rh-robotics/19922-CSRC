package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.HWC;

@Autonomous(name = "AutonomousV2")
public class AutonomousV2 extends OpMode {
    // ------ State Enum ------ //
    private enum State {
        DRIVING_TO_DEPOSIT_PURPLE_PIXEL, DEPOSITING_PURPLE_PIXEL, DRIVING_TO_BACKBOARD, MOVING_AT_BACKBOARD, DEPOSITING_YELLOW_PIXEL, DRIVING_TO_PIXEL_STACK, KNOCKING_PIXEL_STACK, INTAKING_PIXELS, DRIVING_TO_BACKBOARD_2, DELIVERING_BACKBOARD, PARK, STOP
    }

    // ------ Alliance Color Enum ------ //
    private enum AllianceColor {
        RED, BLUE
    }

    // ------ Declare Others ------ //
    private HWC robot;
    private State state = State.DRIVING_TO_DEPOSIT_PURPLE_PIXEL;
    private AllianceColor allianceColor = AllianceColor.RED;
    private HWC.Location elementLocation;
    private String activeTrajectory = "";
    private boolean firstRun = true;

    // ------ Trajectories ------ //
    private Trajectory toDepositCenter;
    private Trajectory toDepositRight;
    private Trajectory toDepositLeft;
    private Trajectory toBackboardFromCenter;
    private Trajectory toBackboardFromRight;
    private Trajectory toBackboardFromLeft;
    private Trajectory toBackboardLeft;
    private Trajectory toBackboardRight;
    private Trajectory toPixelStackFromCenter;
    private Trajectory toPixelStackFromRight;
    private Trajectory toPixelStackFromLeft;
    private Trajectory knockingPixelStack;
    private Trajectory toBackboard2FromPixelStack;
    private Trajectory toParkFromBackboard2;

    // ------ Starting Positions ------ //
    private final Pose2d START_POSE_RED = new Pose2d(12, -60, Math.toRadians(90.00));
    private final Pose2d START_POSE_BLUE = new Pose2d(12, 60, Math.toRadians(270.00));

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

        // ------ Save Element Position ------ //
        elementLocation = robot.detectElement();

        // ------ Alliance Color Selection ------ //
        if (robot.currentGamepad1.a && !robot.previousGamepad1.a) {
            allianceColor = allianceColor.equals(AllianceColor.RED) ? AllianceColor.BLUE : AllianceColor.RED;
        }

        // ------ Telemetry ------ //
        telemetry.addData(">", "Yellow Pixel on Left Side");
        telemetry.addData(">", "Purple Pixel in intake");
        telemetry.addData("Element Location", elementLocation);
        telemetry.addData("Selected Alliance Color", allianceColor);
        telemetry.addData("Status", "Init Loop");
        telemetry.update();
    }

    @Override
    public void start() {
        // ------ Close Claw for Yellow Pixel ------ //
        robot.toggleClaw('L');

        // ------ Set Trajectories based on Alliance Color ------ //
        switch(allianceColor) {
            case RED:
                // ------ Set Robot Start Pose ------ //
                robot.drive.setPoseEstimate(START_POSE_RED);

                // ------ Declare Trajectories ------ //
                // Drive to Center Line
                toDepositCenter = robot.drive.trajectoryBuilder(START_POSE_RED)
                        .lineTo(new Vector2d(12.0, -30))
                        .build();

                // Drive to Right Line
                toDepositRight = robot.drive.trajectoryBuilder(START_POSE_RED)
                        .strafeTo(new Vector2d(23, -47))
                        .build();

                // Drive to Left Line
                toDepositLeft = robot.drive.trajectoryBuilder(START_POSE_RED)
                        .lineToLinearHeading(new Pose2d(9, -40, Math.toRadians(135)))
                        .build();

                // Drive to Backboard from Center
                toBackboardFromCenter = robot.drive.trajectoryBuilder(toDepositCenter.end())
                        .lineToLinearHeading(new Pose2d(44, -35, Math.toRadians(180)))
                        .build();

                // Drive to Backboard from Right
                toBackboardFromRight = robot.drive.trajectoryBuilder(toDepositRight.end())
                        .lineToLinearHeading(new Pose2d(44, -35, Math.toRadians(180)))
                        .build();

                // Drive to Backboard from Left
                toBackboardFromLeft = robot.drive.trajectoryBuilder(toDepositLeft.end())
                        .lineToLinearHeading(new Pose2d(44, -35, Math.toRadians(180)))
                        .build();

                // Drive to Left Side of Backboard
                toBackboardLeft = robot.drive.trajectoryBuilder(toBackboardFromLeft.end())
                        .strafeRight(8)
                        .build();

                // Drive to Right Side of Backboard
                toBackboardRight = robot.drive.trajectoryBuilder(toBackboardFromRight.end())
                        .strafeLeft(8)
                        .build();

                // Drive to Pixel Stack from Center Backboard
                toPixelStackFromCenter = robot.drive.trajectoryBuilder(toBackboardFromCenter.end())
                        .lineToLinearHeading(new Pose2d(44, -35, Math.toRadians(180)))
                        .lineToLinearHeading(new Pose2d(-7, 0, Math.toRadians(180)))
                        .build();

                // Drive to Pixel Stack from Right Backboard
                toPixelStackFromRight = robot.drive.trajectoryBuilder(toBackboardRight.end())
                        .lineToLinearHeading(new Pose2d(44, -35, Math.toRadians(180)))
                        .lineToLinearHeading(new Pose2d(-7, 0, Math.toRadians(180)))
                        .build();

                // Drive to Pixel Stack from Left Backboard
                toPixelStackFromLeft = robot.drive.trajectoryBuilder(toBackboardLeft.end())
                        .lineToLinearHeading(new Pose2d(44, -35, Math.toRadians(180)))
                        .lineToLinearHeading(new Pose2d(-7, 0, Math.toRadians(180)))
                        .build();

                // Knock Pixel Stack
                knockingPixelStack = robot.drive.trajectoryBuilder(toPixelStackFromCenter.end())
                        .lineToLinearHeading(new Pose2d(-58, -26, Math.toRadians(270)))
                        .build();

                // Drive to Backboard 2 from Pixel Stack
                toBackboard2FromPixelStack = robot.drive.trajectoryBuilder(knockingPixelStack.end())
                        .lineToLinearHeading(new Pose2d(-7, 0, Math.toRadians(180)))
                        .lineToLinearHeading(new Pose2d(44, -35, Math.toRadians(180)))
                        .build();

                // Drive to Park from Backboard 2
                toParkFromBackboard2 = robot.drive.trajectoryBuilder(toBackboard2FromPixelStack.end())
                        .strafeLeft(13)
                        .build();

                break;
            case BLUE:
                // ------ Set Robot Start Pose ------ //
                robot.drive.setPoseEstimate(START_POSE_BLUE);

                // ------ Declare Trajectories ------ //

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
            case DRIVING_TO_DEPOSIT_PURPLE_PIXEL:
                drivingToDepositPurplePixel();
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
            case DRIVING_TO_PIXEL_STACK:
                drivingToPixelStack();
                break;
            case KNOCKING_PIXEL_STACK:
                knockingPixelStack();
                break;
            case INTAKING_PIXELS:
                intakingPixels();
                break;
            case DRIVING_TO_BACKBOARD_2:
                drivingToBackboard2();
                break;
            case DELIVERING_BACKBOARD:
                deliveringBackboard();
                break;
            case PARK:
                drivingToPark();
            case STOP:
                stop();
                break;
        }

        // ------ Move Slides Using PID ------ //
        robot.pulleyLComponent.moveUsingPID();
        robot.pulleyRComponent.moveUsingPID();

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
   private void drivingToDepositPurplePixel() {
        // ------ Select Trajectory ------ //
        if(firstRun) {
            firstRun = false;
            if (elementLocation == HWC.Location.CENTER) {
                activeTrajectory = "toDepositCenter";
                robot.drive.followTrajectoryAsync(toDepositCenter);
            } else if (elementLocation == HWC.Location.RIGHT) {
                activeTrajectory = "toDepositRight";
                robot.drive.followTrajectoryAsync(toDepositRight);
            } else {
                activeTrajectory = "toDepositLeft";
                robot.drive.followTrajectoryAsync(toDepositLeft);
            }
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
        robot.intakeMotor.setPower(0.8);
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
            if (elementLocation == HWC.Location.CENTER) {
                activeTrajectory = "toBackboardFromInitial";
                robot.drive.followTrajectoryAsync(toBackboardFromCenter);
            } else if (elementLocation == HWC.Location.RIGHT) {
                activeTrajectory = "toBackboardFromSecond";
                robot.drive.followTrajectoryAsync(toBackboardFromRight);
            } else {
                activeTrajectory = "toBackBoardFromLeft";
                robot.drive.followTrajectoryAsync(toBackboardFromLeft);
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
            if (elementLocation == HWC.Location.RIGHT) {
                activeTrajectory = "toBackboardRight";
                robot.drive.followTrajectoryAsync(toBackboardRight);
            } else if (elementLocation == HWC.Location.LEFT) {
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
        // ------ Move Slides, Passover & Wrist ------ //
        deliver();

        // ------ Wait for Passover to Move ------ //
        robot.elapsedTimeSleep(1000);

        // ------ Open Claw ------ //
        robot.toggleClaw('L');

        // ------ Wait for Claw to Open ------ //
        robot.elapsedTimeSleep(1000);

        // ------ Move Slides, Passover & Wrist ------ //
        intake();

        // ------ Set Next State ------ //
        state = State.PARK;
    }

    // Drive to Pixel Stack
    private void drivingToPixelStack() {
        // ------ Select Trajectory ------ //
        if (firstRun) {
            firstRun = false;
            if (elementLocation == HWC.Location.RIGHT) {
                activeTrajectory = "toPixelStackFromRight";
                robot.drive.followTrajectoryAsync(toPixelStackFromRight);
            } else if (elementLocation == HWC.Location.LEFT) {
                activeTrajectory = "toPixelStackFromLeft";
                robot.drive.followTrajectoryAsync(toPixelStackFromLeft);
            } else {
                activeTrajectory = "toPixelStackFromCenter";
                robot.drive.followTrajectoryAsync(toPixelStackFromCenter);
            }
        }

        // ------ Set Next State ------ //
        if (!robot.drive.isBusy()) {
            state = State.KNOCKING_PIXEL_STACK;
            firstRun = true;
        }
    }

    // Knock Pixel Stack
    private void knockingPixelStack() {
        // ------ Start Trajectory ------ //
        if (firstRun) {
            firstRun = false;
            robot.drive.followTrajectoryAsync(knockingPixelStack);
        }

        // ------ Set Next State ------ //
        if (!robot.drive.isBusy()) {
            state = State.INTAKING_PIXELS;
            firstRun = true;
        }
    }

    // Intake Pixels
    private void intakingPixels() {
        // ------ Run Intake Motor Forward for 1.5 Seconds ------ //
        robot.intakeMotor.setPower(-1);
        robot.elapsedTimeSleep(1500);
        robot.intakeMotor.setPower(0);

        // ------ Set Next State ------ //
        state = State.DRIVING_TO_BACKBOARD_2;
    }

    // Drive to Backboard 2
    private void drivingToBackboard2() {
        // ------ Start Trajectory ------ //
        if (firstRun) {
            firstRun = false;
            robot.drive.followTrajectoryAsync(toBackboard2FromPixelStack);
        }

        // ------ Set Next State ------ //
        if (!robot.drive.isBusy()) {
            state = State.DELIVERING_BACKBOARD;
            firstRun = true;
        }
    }

    // Deliver Backboard
    private void deliveringBackboard() {
        // ------ Move Slides, Passover & Wrist ------ //
        deliver();

        // ------ Wait for Passover to Move ------ //
        robot.elapsedTimeSleep(1000);

        // ------ Open Claw ------ //
        robot.toggleClaw('L');

        // ------ Wait for Claw to Open ------ //
        robot.elapsedTimeSleep(1000);

        // ------ Move Slides, Passover & Wrist ------ //
        intake();

        // ------ Set Next State ------ //
        state = State.PARK;
    }

    // Drive to Park
    private void drivingToPark() {
        // ------ Select Trajectory ------ //
        if(firstRun) {
            firstRun = false;
            robot.drive.followTrajectoryAsync(toParkFromBackboard2);
        }

        // ------ Set Next State ------ //
        if (!robot.drive.isBusy()) {
            state = State.STOP;
            firstRun = true;
        }
    }

    // Method to move to Delivery Position
    private void deliver() {
        robot.passoverArmLeft.setPosition(HWC.passoverDeliveryPos);
        robot.passoverArmRight.setPosition(HWC.passoverDeliveryPos);
        robot.wrist.setPosition(HWC.wristDeliveryPos);
        robot.pulleyLComponent.setTarget(HWC.slidePositions[1]);
        robot.pulleyRComponent.setTarget(HWC.slidePositions[1]);
    }

    // Method to move to Intake Position
    private void intake() {
        robot.passoverArmLeft.setPosition(HWC.passoverIntakePos);
        robot.passoverArmRight.setPosition(HWC.passoverIntakePos);
        robot.wrist.setPosition(HWC.wristIntakePos);
        robot.pulleyLComponent.setTarget(HWC.slidePositions[0]);
        robot.pulleyRComponent.setTarget(HWC.slidePositions[0]);
    }
}