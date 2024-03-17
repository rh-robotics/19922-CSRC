package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.auton.enums.AllianceColor;
import org.firstinspires.ftc.teamcode.subsystems.HWC;
import org.firstinspires.ftc.teamcode.subsystems.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous(name = "FarAutonomousV2")
public class FarAutonomousV2 extends OpMode {
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
    private TrajectorySequence toDepositCenter;
    private TrajectorySequence toDepositRight;
    private TrajectorySequence toDepositLeft;
    private Trajectory toPixelStackFromCenter;
    private Trajectory toPixelStackFromRight;
    private Trajectory toPixelStackFromLeft;
    private Trajectory knockingPixelStack;
    private Trajectory intakingPixels1;
    private Trajectory intakingPixels2;
    private Trajectory intakingPixels3;
    private TrajectorySequence toBackboardFromCenter;
    private TrajectorySequence toBackboardFromRight;
    private TrajectorySequence toBackboardFromLeft;
    private TrajectorySequence toBackboardFromPixelStack;
    private Trajectory toPark;

    // ------ Starting Positions ------ //
    private final Pose2d START_POSE_RED = new Pose2d(-36, -60, Math.toRadians(90.00));
    private final Pose2d START_POSE_BLUE = new Pose2d(-36, 60, Math.toRadians(270.00));

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

        // ------ Reset Slide Encoders ------ //
        resetSlideEncoders();

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
            if (allianceColor.equals(AllianceColor.RED)) {
                allianceColor = AllianceColor.BLUE;
            } else {
                allianceColor = AllianceColor.RED;
            }
        }

        // ------ Set Trajectories based on Alliance Color ------ //
        switch (allianceColor) {
            case RED:
                // ------ Set Robot Start Pose ------ //
                robot.drive.setPoseEstimate(START_POSE_RED);

                // ------ Declare Trajectories ------ //
                // Drive to Center Line
                toDepositCenter = robot.drive.trajectorySequenceBuilder(START_POSE_RED)
                        .strafeTo(new Vector2d(12.0 - 48, -34))
                        .setReversed(true)
                        .strafeTo(new Vector2d(12.0 - 48, -40))
                        .setReversed(false)
                        //.strafeTo(new Vector2d(12.0-48, -34-5))
                        .build();

                // Drive to Right Line
                toDepositRight = robot.drive.trajectorySequenceBuilder(START_POSE_RED)
                        .splineToLinearHeading(new Pose2d(18 - 48, -35, Math.toRadians(45)), Math.toRadians(45))
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(18 - 48 - 5, -40, Math.toRadians(45)), Math.toRadians(180))
                        .setReversed(false)
                        //.back(5)
                        .build();

                // Drive to Left Line
                toDepositLeft = robot.drive.trajectorySequenceBuilder(START_POSE_RED)
                        .strafeTo(new Vector2d(7 - 48, -39))
                        .setReversed(true)
                        .strafeTo(new Vector2d(12.0 - 48, -45))
                        .setReversed(false)
                        //.strafeTo(new Vector2d(7.0-48, -39-5))
                        .build();

                // Drive to Pixel Stack from Center
                toPixelStackFromCenter = robot.drive.trajectoryBuilder(toDepositCenter.end())
                        .lineToLinearHeading(new Pose2d(-57, -40, Math.toRadians(180 - 40)))
                        .build();

                // Drive to Pixel Stack from Right
                // TODO: Will Likely Hit Purple Pixel, Adjust Trajectory after Testing
                toPixelStackFromRight = robot.drive.trajectoryBuilder(toDepositRight.end())
                        .lineToLinearHeading(new Pose2d(-57, -40, Math.toRadians(180 - 40)))
                        .build();

                // Drive to Pixel Stack from Left
                toPixelStackFromLeft = robot.drive.trajectoryBuilder(toDepositLeft.end())
                        .lineToLinearHeading(new Pose2d(-57, -40, Math.toRadians(180 - 40)))
                        .build();

                // Knock Pixel Stack
                knockingPixelStack = robot.drive.trajectoryBuilder(toPixelStackFromCenter.end())
                        .splineToLinearHeading(new Pose2d(-59, -36, Math.toRadians(180 - 40)), Math.toRadians(180 - 40))
                        .splineToLinearHeading(new Pose2d(-57, -30, Math.toRadians(180 - 40)), Math.toRadians(180 - 0))
                        .build();

                // Intake Pixels (1)
                intakingPixels1 = robot.drive.trajectoryBuilder(knockingPixelStack.end())
                        .strafeRight(8)
                        //.splineTo(new Vector2d(-63, -36), Math.toRadians(120))
                        .addDisplacementMarker(() -> robot.drive.followTrajectoryAsync(intakingPixels2))
                        .build();

//                // Intake Pixels (2)
                intakingPixels2 = robot.drive.trajectoryBuilder(intakingPixels1.end())
                        .strafeLeft(2)
//                        .addDisplacementMarker(() -> robot.drive.followTrajectoryAsync(intakingPixels3))
                        .build();

                // Intake Pixels (3)
//                intakingPixels3 = robot.drive.trajectoryBuilder(intakingPixels2.end())
//                        .strafeRight(5)
//                        .build();

                // Drive to Backboard from Center
                toBackboardFromCenter = robot.drive.trajectorySequenceBuilder(intakingPixels2.end())
                        .setReversed(true)
                        .splineTo(new Vector2d(-50, -12), Math.toRadians(0))
                        .splineTo(new Vector2d(-12, -8), Math.toRadians(0))
                        .splineTo(new Vector2d(32, -12), Math.toRadians(0))
                        .splineTo(new Vector2d(49, -34), Math.toRadians(0))
                        .build();

                // Drive to Backboard from Right
                toBackboardFromRight = robot.drive.trajectorySequenceBuilder(intakingPixels2.end())
                        .setReversed(true)
                        .splineTo(new Vector2d(-50, -12), Math.toRadians(0))
                        .splineTo(new Vector2d(-12, -8), Math.toRadians(0))
                        .splineTo(new Vector2d(32, -12), Math.toRadians(0))
                        .splineTo(new Vector2d(49, -40), Math.toRadians(0))
                        .build();

                // Drive to Backboard from Left
                toBackboardFromLeft = robot.drive.trajectorySequenceBuilder(intakingPixels2.end())
                        .setReversed(true)
                        .splineTo(new Vector2d(-50, -12), Math.toRadians(0))
                        .splineTo(new Vector2d(-12, -8), Math.toRadians(0))
                        .splineTo(new Vector2d(32, -12), Math.toRadians(0))
                        .splineTo(new Vector2d(49, -28), Math.toRadians(0))
                        .build();

                // Drive to Park
                toPark = robot.drive.trajectoryBuilder(toBackboardFromCenter.end())
                        .strafeTo(new Vector2d(48, -60))
                        .build();

//                // Drive to Backboard from Pixel Stack (DEPRECATED?)
//                toBackboardFromPixelStack = robot.drive.trajectorySequenceBuilder(intakingPixels2.end())
//                        .setReversed(true)
//                        .splineTo(new Vector2d(-12, -60), Math.toRadians(0))
//                        .splineTo(new Vector2d(47, -35), Math.toRadians(0))
//                        .build();

                break;
            case BLUE:
                // ------ Set Robot Start Pose ------ //
                robot.drive.setPoseEstimate(START_POSE_BLUE);

                // ------ Declare Trajectories ------ //
                // Drive to Center Line
                toDepositCenter = robot.drive.trajectorySequenceBuilder(START_POSE_BLUE)
                        .strafeTo(new Vector2d(12.0 - 48, 34))
                        .setReversed(true)
                        .strafeTo(new Vector2d(12.0 - 48, 40))
                        .setReversed(false)
                        //.strafeTo(new Vector2d(12.0-48, -34-5))
                        .build();

                // Drive to Right Line
                toDepositRight = robot.drive.trajectorySequenceBuilder(START_POSE_BLUE)
                        .strafeTo(new Vector2d(7 - 48, 39))
                        .setReversed(true)
                        .strafeTo(new Vector2d(7 - 48, 45))
                        .setReversed(false)

                        //.back(5)
                        .build();

                // Drive to Left Line
                toDepositLeft = robot.drive.trajectorySequenceBuilder(START_POSE_BLUE)
                        .splineToLinearHeading(new Pose2d(18 - 48, 35, Math.toRadians(360 - 45)), Math.toRadians(360 - 45))
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(18 - 48 - 5, 40, Math.toRadians(360 - 45)), Math.toRadians(360 - 45))
                        .setReversed(false)
                        .build();

                // Drive to Pixel Stack from Center
                toPixelStackFromCenter = robot.drive.trajectoryBuilder(toDepositCenter.end())
                        .lineToLinearHeading(new Pose2d(-57, 40, Math.toRadians(180 + 40)))
                        .build();

                // Drive to Pixel Stack from Right
                // TODO: Will Likely Hit Purple Pixel, Adjust Trajectory after Testing
                toPixelStackFromRight = robot.drive.trajectoryBuilder(toDepositRight.end())
                        .lineToLinearHeading(new Pose2d(-57, 40, Math.toRadians(180 + 40)))
                        .build();

                // Drive to Pixel Stack from Left
                toPixelStackFromLeft = robot.drive.trajectoryBuilder(toDepositLeft.end())
                        .lineToLinearHeading(new Pose2d(-57, 40, Math.toRadians(180 + 40)))
                        .build();

                // Knock Pixel Stack
                knockingPixelStack = robot.drive.trajectoryBuilder(toPixelStackFromCenter.end())
                        .splineToLinearHeading(new Pose2d(-59, 36, Math.toRadians(180 + 40)), Math.toRadians(180 + 40))
                        .splineToLinearHeading(new Pose2d(-57, 30, Math.toRadians(180 + 40)), Math.toRadians(180 - 0))
                        .build();

                // Intake Pixels (1)
                intakingPixels1 = robot.drive.trajectoryBuilder(knockingPixelStack.end())
                        .strafeLeft(8)
                        //.splineTo(new Vector2d(-63, -36), Math.toRadians(120))
                        .addDisplacementMarker(() -> robot.drive.followTrajectoryAsync(intakingPixels2))
                        .build();

//                // Intake Pixels (2)
                intakingPixels2 = robot.drive.trajectoryBuilder(intakingPixels1.end())
                        .strafeRight(2)
//                        .addDisplacementMarker(() -> robot.drive.followTrajectoryAsync(intakingPixels3))
                        .build();

                // Intake Pixels (3)
//                intakingPixels3 = robot.drive.trajectoryBuilder(intakingPixels2.end())
//                        .strafeRight(5)
//                        .build();

                // Drive to Backboard from Center
                toBackboardFromCenter = robot.drive.trajectorySequenceBuilder(intakingPixels2.end())
                        .setReversed(true)
                        .splineTo(new Vector2d(-50, 12), Math.toRadians(0))
                        .splineTo(new Vector2d(-12, 8), Math.toRadians(0))
                        .splineTo(new Vector2d(32, 12), Math.toRadians(0))
                        .splineTo(new Vector2d(49, 34), Math.toRadians(0))
                        .build();

                // Drive to Backboard from Right
                toBackboardFromRight = robot.drive.trajectorySequenceBuilder(intakingPixels2.end())
                        .setReversed(true)
                        .splineTo(new Vector2d(-50, 12), Math.toRadians(0))
                        .splineTo(new Vector2d(-12, 8), Math.toRadians(0))
                        .splineTo(new Vector2d(32, 12), Math.toRadians(0))
                        .splineTo(new Vector2d(49, 28), Math.toRadians(0))
                        .build();

                // Drive to Backboard from Left
                toBackboardFromLeft = robot.drive.trajectorySequenceBuilder(intakingPixels2.end())
                        .setReversed(true)
                        .splineTo(new Vector2d(-50, 12), Math.toRadians(0))
                        .splineTo(new Vector2d(-12, 8), Math.toRadians(0))
                        .splineTo(new Vector2d(32, 12), Math.toRadians(0))
                        .splineTo(new Vector2d(49, 40), Math.toRadians(0))
                        .build();

                // Drive to Park
                toPark = robot.drive.trajectoryBuilder(toBackboardFromCenter.end())
                        .strafeTo(new Vector2d(48, 60))
                        .build();

//                // Drive to Backboard from Pixel Stack (DEPRECATED?)
//                toBackboardFromPixelStack = robot.drive.trajectorySequenceBuilder(intakingPixels2.end())
//                        .setReversed(true)
//                        .splineTo(new Vector2d(-12, -60), Math.toRadians(0))
//                        .splineTo(new Vector2d(47, -35), Math.toRadians(0))
//                        .build();

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
//            case DEPOSITING_YELLOW_PIXEL:
//                depositingYellowPixel();
//                break;
            case DRIVING_TO_PIXEL_STACK:
                drivingToPixelStack();
                break;
            case KNOCKING_PIXEL_STACK:
                knockingPixelStack();
                break;
            case INTAKING_PIXELS:
                intakingPixels();
                break;
//            case DRIVING_TO_BACKBOARD_2:
//                drivingToBackboard2();
//                break;
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
        if (firstRun) {
            firstRun = false;
            if (elementLocation == HWC.Location.CENTER) {
                activeTrajectory = "toDepositCenter";
                robot.drive.followTrajectorySequenceAsync(toDepositCenter);
            } else if (elementLocation == HWC.Location.RIGHT) {
                activeTrajectory = "toDepositRight";
                robot.drive.followTrajectorySequenceAsync(toDepositRight);
            } else {
                activeTrajectory = "toDepositLeft";
                robot.drive.followTrajectorySequenceAsync(toDepositLeft);
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
        state = State.DRIVING_TO_PIXEL_STACK;
    }

    // Drive to Pixel Stack
    private void drivingToPixelStack() {
        // ------ Select Trajectory ------ //
        if (firstRun) {
            firstRun = false;
            if (elementLocation == HWC.Location.CENTER) {
                activeTrajectory = "toPixelStackFromCenter";
                robot.drive.followTrajectoryAsync(toPixelStackFromCenter);
            } else if (elementLocation == HWC.Location.RIGHT) {
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
        // ------ Run Intake ------ //
        robot.intakeMotor.setPower(-1);

        // ------ Start Trajectory ------ //
        if (firstRun) {
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

        // ------ Run Intake ------ //
        robot.intakeMotor.setPower(-1);

        // ------ Start Trajectory ------ //
        if (firstRun) {
            firstRun = false;
            activeTrajectory = "intakingPixels";
            robot.drive.followTrajectoryAsync(intakingPixels1);
        }

        // ------ Check Claws & Close ------ //
        checkClaws();

        // ------ Set Next State ------ //
        if (!robot.drive.isBusy()) {
            state = State.DRIVING_TO_BACKBOARD;
            firstRun = true;
        }
    }

    // Drive to Backboard
    private void drivingToBackboard() {
        // ------ Select Trajectory ------ //
        if (firstRun) {
            firstRun = false;
            if (elementLocation == HWC.Location.CENTER) {
                activeTrajectory = "toBackboardFromInitial";
                robot.drive.followTrajectorySequenceAsync(toBackboardFromCenter);
            } else if (elementLocation == HWC.Location.RIGHT) {
                activeTrajectory = "toBackboardFromSecond";
                robot.drive.followTrajectorySequenceAsync(toBackboardFromRight);
            } else {
                activeTrajectory = "toBackBoardFromLeft";
                robot.drive.followTrajectorySequenceAsync(toBackboardFromLeft);
            }
        }

        // ------ Check Claws Again & Close ------ //
        checkClaws();

        // ------ Set Next State ------ //
        if (!robot.drive.isBusy()) {
            state = State.DELIVERING_BACKBOARD;
            firstRun = true;
        }
    }

    // Deliver Backboard
    private void deliveringBackboard() {
        // ------ Close Claws if not closed already ------ //
        robot.clawL.setPosition(0.5);
        robot.clawR.setPosition(0.5);

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
        if (firstRun) {
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

    // Drive to Backboard
//    private void drivingToBackboard2() {
//        // ------ Select Trajectory ------ //
//        if(firstRun) {
//            firstRun = false;
//            activeTrajectory = "toBackboardFromPixelStack";
//            robot.drive.followTrajectorySequence(toBackboardFromPixelStack);
//        }
//
//        // ------ Check Claws While Driving ------ //
//        checkClaws();
//
//        // ------ Set Next State ------ //
//        if (!robot.drive.isBusy()) {
//            state = State.DELIVERING_BACKBOARD;
//            firstRun = true;
//        }
//    }
//
//    // Deposit Yellow Pixel
//    private void depositingYellowPixel() {
//        // ------ Move Slides, Passover & Wrist ------ //
//        deliver(-170);
//
//        // ------ Wait for Passover to Move ------ //
//        robot.elapsedTimeSleep(1000);
//
//        // ------ Open Claw ------ //
//        robot.clawL.setPosition(1);
//        robot.clawR.setPosition(0);
//
//        // ------ Wait for Claw to Open ------ //
//        robot.elapsedTimeSleep(1000);
//
//        // ------ Move Slides, Passover & Wrist ------ //
//        intake();
//
//        // ------ Set Next State ------ //
//        state = State.DRIVING_TO_PIXEL_STACK;
//        firstRun = true;
//    }

    // ------ Helper Methods Used All Around ------ //
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
        boolean leftFull = false;
        boolean rightFull = false;
        if (robot.colorLeft.getDistance(DistanceUnit.CM) <= 1.25) {
            robot.clawL.setPosition(0.5);
            leftFull = true;
        }

        if (robot.colorRight.getDistance(DistanceUnit.CM) <= 2) {
            robot.clawR.setPosition(0.5);
            rightFull = true;
        }

        // If both Pixels are Detected, Reverse Intake
        if (leftFull && rightFull) {
            robot.intakeMotor.setPower(1);
        }
    }

    // Method to Reset Slide Encoders
    private void resetSlideEncoders() {
        robot.leftPulley.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightPulley.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftPulley.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightPulley.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}