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
    private Trajectory intakingPixels;
    private Trajectory intakingPixelsSweep;
    private Trajectory intakingPixelsTurnBack;
    private TrajectorySequence toBackboardFromPixelStack;
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

        // ------ Close Claw for Yellow Pixel ------ //
        robot.toggleClaw('L');

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
                        .lineToLinearHeading(new Pose2d(49, -30, Math.toRadians(180)))
                        .build();

                // Drive to Pixel Stack from Center Backboard
                toPixelStackFromCenter = robot.drive.trajectoryBuilder(toBackboardFromCenter.end())
                        .splineTo(new Vector2d(-1, -9), Math.toRadians(180))
                        .splineTo(new Vector2d(-60, -24), Math.toRadians(180))
                        .build();

                // Drive to Pixel Stack from Right Backboard
                toPixelStackFromRight = robot.drive.trajectoryBuilder(toBackboardFromRight.end())
                        .splineTo(new Vector2d(-1, -9), Math.toRadians(180))
                        .splineTo(new Vector2d(-60, -24), Math.toRadians(180))
                        .build();

                // Drive to Pixel Stack from Left Backboard
                toPixelStackFromLeft = robot.drive.trajectoryBuilder(toBackboardFromLeft.end())
                        .splineTo(new Vector2d(-1, -9), Math.toRadians(180))
                        .splineTo(new Vector2d(-60, -24), Math.toRadians(180))
                        .build();

                // Knock Pixel Stack
                knockingPixelStack = robot.drive.trajectoryBuilder(toPixelStackFromCenter.end())
                        .splineToLinearHeading(new Pose2d(-60, -25, Math.toRadians(270)), Math.toRadians(270))
                        .build();

                // Intake Pixels
                intakingPixels = robot.drive.trajectoryBuilder(knockingPixelStack.end())
                        .splineToLinearHeading(new Pose2d(-60, -24, Math.toRadians(180)), Math.toRadians(180))
                        .addDisplacementMarker(() -> robot.drive.followTrajectoryAsync(intakingPixelsSweep))
                        .build();

                // Intaking Pixels Sweep
                intakingPixelsSweep = robot.drive.trajectoryBuilder(intakingPixels.end())
                        .splineToLinearHeading(new Pose2d(-60, -25, Math.toRadians(270)), Math.toRadians(270))
                        .addDisplacementMarker(() -> robot.drive.followTrajectoryAsync(intakingPixelsTurnBack))
                        .build();

                // Sweep Backwards
                intakingPixelsTurnBack = robot.drive.trajectoryBuilder(intakingPixelsSweep.end())
                        .splineToLinearHeading(new Pose2d(-60, -24, Math.toRadians(180)), Math.toRadians(180))
                        .build();

                // Drive to Backboard 2 from Pixel Stack
                toBackboardFromPixelStack = robot.drive.trajectorySequenceBuilder(knockingPixelStack.end())
                        .setReversed(true)
                        .splineTo(new Vector2d(0, -9), Math.toRadians(0))
                        .splineTo(new Vector2d(48, -35), Math.toRadians(0))
                        .build();

                // Drive to Park from Backboard 2
                toParkFromBackboard2 = robot.drive.trajectoryBuilder(toBackboardFromPixelStack.end())
                        .strafeTo(new Vector2d(48, -60))
                        .build();

                break;
            case BLUE:
                // ------ Set Robot Start Pose ------ //
                robot.drive.setPoseEstimate(START_POSE_BLUE);

                // ------ Declare Trajectories ------ //
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
        state = State.DRIVING_TO_PIXEL_STACK;
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
        // ----- Start Trajectory ------ //
        if (firstRun) {
            firstRun = false;
            robot.time.reset();
            activeTrajectory = "intakingPixels";
            robot.drive.followTrajectoryAsync(intakingPixels);
        }

        // ------ Run Intake Motor Forward------ //
        robot.intakeMotor.setPower(-1);

        // ------ Close Claw on Color Sensor ------ //
        checkClaws();

        // ------ Set Next State ------ //
        if(!robot.drive.isBusy() && robot.time.seconds() >= 5) {
            state = State.DRIVING_TO_BACKBOARD_2;
            firstRun = true;
        }
    }

    // Drive to Backboard 2
    private void drivingToBackboard2() {
        // ------ Start Trajectory ------ //
        if (firstRun) {
            firstRun = false;
            activeTrajectory = "toBackboard2FromPixelStack";
            robot.drive.followTrajectorySequenceAsync(toBackboardFromPixelStack);
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
            activeTrajectory = "toParkFromBackboard2";
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
        robot.pulleyLComponent.setTarget(-170);
        robot.pulleyRComponent.setTarget(-170);
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