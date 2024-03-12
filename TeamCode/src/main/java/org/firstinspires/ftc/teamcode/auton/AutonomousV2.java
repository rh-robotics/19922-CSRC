package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.auton.enums.AllianceColor;
import org.firstinspires.ftc.teamcode.subsystems.HWC;
import org.firstinspires.ftc.teamcode.subsystems.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous(name = "AutonomousV2")
public class AutonomousV2 extends OpMode {
    // ------ State Enum ------ //
    private enum State {
        DRIVING_TO_DEPOSIT_PURPLE_PIXEL, DEPOSITING_PURPLE_PIXEL, DRIVING_TO_BACKBOARD, DEPOSITING_YELLOW_PIXEL, DRIVING_TO_PIXEL_STACK, KNOCKING_PIXEL_STACK, INTAKING_PIXELS, DRIVING_TO_BACKBOARD_2, DELIVERING_BACKBOARD, PARK, STOP
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
    private Trajectory toPixelStackFromCenter;
    private Trajectory toPixelStackFromRight;
    private Trajectory toPixelStackFromLeft;
    private Trajectory knockingPixelStack;
    private Trajectory intakingPixels1;
    private Trajectory intakingPixels2;
    private Trajectory intakingPixels3;
    private TrajectorySequence toBackboardFromPixelStack;
    private Trajectory toPark;

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

        // ------ Close Claw for Yellow Pixel ------ //
        robot.clawL.setPosition(0.5);

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
        elementLocation = robot.detectElement(allianceColor);

        // ------ Alliance Color Selection ------ //
        if (robot.currentGamepad1.a && !robot.previousGamepad1.a) {
            allianceColor = allianceColor.equals(AllianceColor.RED) ? AllianceColor.BLUE : AllianceColor.RED;
        }

        // ------ Set Trajectories based on Alliance Color ------ //
        switch(allianceColor) {
            case RED:
                // ------ Set Robot Start Pose ------ //
                robot.drive.setPoseEstimate(START_POSE_RED);

                // ------ Declare Trajectories ------ //
                // Drive to Center Line
                toDepositCenter = robot.drive.trajectoryBuilder(START_POSE_RED)
                        .lineTo(new Vector2d(12.0, -34))
                        .build();

                // Drive to Right Line
                toDepositRight = robot.drive.trajectoryBuilder(START_POSE_RED)
                        .strafeTo(new Vector2d(22, -43))
                        .build();

                // Drive to Left Line
                toDepositLeft = robot.drive.trajectoryBuilder(START_POSE_RED)
                        .splineToLinearHeading(new Pose2d(7, -39, Math.toRadians(135)), Math.toRadians(135))
                        .build();

                // Drive to Backboard from Center
                toBackboardFromCenter = robot.drive.trajectoryBuilder(toDepositCenter.end())
                        .lineToLinearHeading(new Pose2d(49, -35, Math.toRadians(180)))
                        .build();

                // Drive to Backboard from Right
                toBackboardFromRight = robot.drive.trajectoryBuilder(toDepositRight.end())
                        .lineToLinearHeading(new Pose2d(49, -37, Math.toRadians(180)))
                        .build();

                // Drive to Backboard from Left
                toBackboardFromLeft = robot.drive.trajectoryBuilder(toDepositLeft.end())
                        .lineToLinearHeading(new Pose2d(49, -28, Math.toRadians(180)))
                        .build();

                // Drive to Pixel Stack from Center
                toPixelStackFromCenter = robot.drive.trajectoryBuilder(toBackboardFromCenter.end())
                        .splineTo(new Vector2d(-3, -8), Math.toRadians(180))
                        .splineTo(new Vector2d(-65, -17), Math.toRadians(180))
                        .build();

                // Drive to Pixel Stack from Right
                // TODO: Will Likely Hit Purple Pixel, Adjust Trajectory after Testing
                toPixelStackFromRight = robot.drive.trajectoryBuilder(toBackboardFromRight.end())
                        .splineTo(new Vector2d(-3, -8), Math.toRadians(180))
                        .splineTo(new Vector2d(-65, -17), Math.toRadians(180))
                        .build();

                // Drive to Pixel Stack from Left
                toPixelStackFromLeft = robot.drive.trajectoryBuilder(toBackboardFromLeft.end())
                        .splineTo(new Vector2d(-3, -8), Math.toRadians(180))
                        .splineTo(new Vector2d(-65, -17), Math.toRadians(180))
                        .build();

                // Knock Pixel Stack
                knockingPixelStack = robot.drive.trajectoryBuilder(toPixelStackFromCenter.end())
                        .splineToLinearHeading(new Pose2d(-65, -18, Math.toRadians(220)), Math.toRadians(220))
                        .build();

                // Intake Pixels (1)
                intakingPixels1 = robot.drive.trajectoryBuilder(knockingPixelStack.end())
                        .splineToLinearHeading(new Pose2d(-60, -20, Math.toRadians(180)), Math.toRadians(180))
                        .addDisplacementMarker(() -> robot.drive.followTrajectoryAsync(intakingPixels2))
                        .build();

                // Intake Pixels (2)
                // TODO: Will likely not be enough strafe, adjust as needed
                intakingPixels2 = robot.drive.trajectoryBuilder(intakingPixels1.end())
                        .strafeLeft(5)
                        .addDisplacementMarker(() -> robot.drive.followTrajectoryAsync(intakingPixels3))
                        .build();

                // Intake Pixels (3)
                // TODO: Will likely not be enough strafe, adjust as needed
                intakingPixels3 = robot.drive.trajectoryBuilder(intakingPixels2.end())
                        .strafeRight(5)
                        .build();

                // Drive to Backboard from Pixel Stack
                toBackboardFromPixelStack = robot.drive.trajectorySequenceBuilder(intakingPixels3.end())
                        .setReversed(true)
                        .splineTo(new Vector2d(-3, -8), Math.toRadians(0))
                        .splineTo(new Vector2d(49, -35), Math.toRadians(0))

                        .build();

                // Drive to Park
                toPark = robot.drive.trajectoryBuilder(toBackboardFromPixelStack.end())
                        .strafeTo(new Vector2d(48, -60))
                        .build();

                break;
            case BLUE:
                // ------ Set Robot Start Pose ------ //
                robot.drive.setPoseEstimate(START_POSE_BLUE);

                // ------ Declare Trajectories ------ //
                // Drive to Center Line
                toDepositCenter = robot.drive.trajectoryBuilder(START_POSE_BLUE)
                        .lineTo(new Vector2d(12.0, 34))
                        .build();

                // Drive to Right Line
                toDepositLeft = robot.drive.trajectoryBuilder(START_POSE_BLUE)
                        .strafeTo(new Vector2d(23, 45))
                        .build();

                // Drive to Left Line
                toDepositRight = robot.drive.trajectoryBuilder(START_POSE_BLUE)
                        .splineToLinearHeading(new Pose2d(7, 39, Math.toRadians(225)), Math.toRadians(225))
                        .build();

                // Drive to Backboard from Center
                toBackboardFromCenter = robot.drive.trajectoryBuilder(toDepositCenter.end())
                        .lineToLinearHeading(new Pose2d(49, 35, Math.toRadians(180)))
                        .build();

                // Drive to Backboard from Right
                toBackboardFromLeft = robot.drive.trajectoryBuilder(toDepositLeft.end())
                        .lineToLinearHeading(new Pose2d(49, 41, Math.toRadians(180)))
                        .build();

                // Drive to Backboard from Left
                toBackboardFromRight = robot.drive.trajectoryBuilder(toDepositRight.end())
                        .lineToLinearHeading(new Pose2d(49, 30, Math.toRadians(180)))
                        .build();

                // TODO: Once Red is tested, copy trajectories with reverse y values and angles, then test.
                // Drive to Pixel Stack from Center

                // Drive to Pixel Stack from Right

                // Drive to Pixel Stack from Left

                // Knock Pixel Stack

                // Intake Pixels (1)

                // Intake Pixels (2)

                // Intake Pixels (3)

                // Drive to Backboard from Pixel Stack

                // Drive to Park

                break;
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
            state = State.DEPOSITING_YELLOW_PIXEL;
            firstRun = true;
        }
    }

    // Deposit Yellow Pixel
    private void depositingYellowPixel() {
        // ------ Move Slides, Passover & Wrist ------ //
        deliver(-170);

        // ------ Wait for Passover to Move ------ //
        robot.elapsedTimeSleep(1000);

        // ------ Open Claw ------ //
        robot.clawL.setPosition(1);
        robot.clawR.setPosition(0);

        // ------ Wait for Claw to Open ------ //
        robot.elapsedTimeSleep(1000);

        // ------ Move Slides, Passover & Wrist ------ //
        intake();

        // ------ Set Next State ------ //
        state = State.DRIVING_TO_PIXEL_STACK;
        firstRun = true;
    }

    // Drive to Pixel Stack
    private void drivingToPixelStack() {
        // ------ Select Trajectory ------ //
        if(firstRun) {
            firstRun = false;
            if(elementLocation == HWC.Location.CENTER) {
                activeTrajectory = "toPixelStackFromCenter";
                robot.drive.followTrajectoryAsync(toPixelStackFromCenter);
            } else if(elementLocation == HWC.Location.RIGHT) {
                activeTrajectory = "toPixelStackFromRight";
                robot.drive.followTrajectoryAsync(toPixelStackFromRight);
            } else {
                activeTrajectory = "toPixelStackFromLeft";
                robot.drive.followTrajectoryAsync(toPixelStackFromLeft);
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
        if(firstRun) {
            firstRun = false;
            activeTrajectory = "knockingPixelStack";
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
        // ------ Start Trajectory ------ //
        if(firstRun) {
            firstRun = false;
            activeTrajectory = "intakingPixels";
            robot.drive.followTrajectoryAsync(intakingPixels1);
        }

        // ------ Run Intake ------ //
        robot.intakeMotor.setPower(-1);

        // ------ Check Claws & Close ------ //
        checkClaws();

        // ------ Set Next State ------ //
        if (!robot.drive.isBusy()) {
            state = State.DRIVING_TO_BACKBOARD_2;
            firstRun = true;
        }
    }

    // Drive to Backboard
    private void drivingToBackboard2() {
        // ------ Select Trajectory ------ //
        if(firstRun) {
            firstRun = false;
            activeTrajectory = "toBackboardFromPixelStack";
            robot.drive.followTrajectorySequence(toBackboardFromPixelStack);
        }

        // ------ Check Claws While Driving ------ //
        checkClaws();

        // ------ Set Next State ------ //
        if (!robot.drive.isBusy()) {
            state = State.DELIVERING_BACKBOARD;
            firstRun = true;
        }
    }

    // Deliver Backboard
    private void deliveringBackboard() {
        // ------ If Intake is on Turn it Off ------ //
        robot.intakeMotor.setPower(0);

        // ------ Move Slides, Passover & Wrist ------ //
        deliver(HWC.slidePositions[2]);

        // ------ Wait for Passover to Move ------ //
        robot.elapsedTimeSleep(1000);

        // ------ Open Claw ------ //
        robot.clawL.setPosition(1);
        robot.clawR.setPosition(0);

        // ------ Wait for Claw to Open ------ //
        robot.elapsedTimeSleep(1000);

        // ------ Move Slides, Passover & Wrist ------ //
        intake();

        // ------ Set Next State ------ //
        state = State.PARK;
        firstRun = true;
    }

    // Drive to Park
    private void drivingToPark() {
        // ------ Select Trajectory ------ //
        if(firstRun) {
            firstRun = false;
            activeTrajectory = "toPark";
            robot.drive.followTrajectoryAsync(toPark);
        }

        // ------ Set Next State ------ //
        if (!robot.drive.isBusy()) {
            state = State.STOP;
            firstRun = true;
        }
    }

    // ------ Quick Methods Used All Around ------ //
    // Method to move to Delivery Position
    private void deliver(int slideHeight) {
        robot.passoverArmLeft.setPosition(HWC.passoverDeliveryPos);
        robot.passoverArmRight.setPosition(HWC.passoverDeliveryPos);
        robot.wrist.setPosition(HWC.wristDeliveryPos);
        robot.pulleyLComponent.setTarget(slideHeight);
        robot.pulleyRComponent.setTarget(slideHeight);
    }

    // Method to move to Intake Position
    private void intake() {
        robot.passoverArmLeft.setPosition(HWC.passoverIntakePos);
        robot.passoverArmRight.setPosition(HWC.passoverIntakePos);
        robot.wrist.setPosition(HWC.wristIntakePos);
        robot.pulleyLComponent.setTarget(HWC.slidePositions[0]);
        robot.pulleyRComponent.setTarget(HWC.slidePositions[0]);
    }

    // Method to Check Claws & Close
    private void checkClaws() {
        // Close Claws when Pixel Detected
        if (robot.colorLeft.getDistance(DistanceUnit.CM) <= 2) {
            robot.clawL.setPosition(0.5);
        }

        if (robot.colorRight.getDistance(DistanceUnit.CM) <= 2) {
            robot.clawR.setPosition(0.5);
        }

        // If both Pixels are Detected, Stop Intake
        if (robot.colorLeft.getDistance(DistanceUnit.CM) <= 1 && robot.colorRight.getDistance(DistanceUnit.CM) <= 1) {
            robot.intakeMotor.setPower(0);
        }
    }
}