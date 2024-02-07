package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.HWC;
import org.firstinspires.ftc.teamcode.subsystems.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous(name = "AutonomousV1")
public class AutonomousV1 extends OpMode {
    private enum State {
        DRIVING_TO_DETECT_INITIAL, DRIVING_TO_DETECT_SECOND, DEPOSITING_PURPLE_PIXEL, DRIVING_TO_BACKBOARD, DEPOSITING_YELLOW_PIXEL,
    }

    private enum TeamElementLocation {
        CENTER, LEFT, RIGHT;

        public boolean isDeferred() {
            return this == RIGHT;
        }
    }

    // ------ Variables ------ //
    private TeamElementLocation teamElementLocation;
    private State state = State.DRIVING_TO_DETECT_INITIAL;
    private HWC robot;

    // ------ Trajectories ------ //
    private TrajectorySequence drivingToInitialDetectionTrajectory;
    private TrajectorySequence drivingToSecondDetectionTrajectory;
    private TrajectorySequence drivingToLastResortTrajectory;
    private TrajectorySequence drivingToBackboardTrajectory;

    @Override
    public void init() {
        // ------ Telemetry ------ //
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        // ------ Initialize Robot Hardware ------ //
        robot = new HWC(hardwareMap, telemetry);

        // ------ Get Trajectories All Good ------ //
        drivingToInitialDetectionTrajectory = robot.drive.trajectorySequenceBuilder(new Pose2d(11.84, -60.75, Math.toRadians(90.00))).UNSTABLE_addTemporalMarkerOffset(0.68, () -> {
        }).splineTo(new Vector2d(11.67, -44.39), Math.toRadians(90.79)).build();

        // ------ CV ------ //
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
    public void start() {
        // ------ Close Claw for Yellow Pixel ------ //
        robot.toggleClaw('L');
    }

    @Override
    public void loop() {
        switch (state) {
            case DRIVING_TO_DETECT_INITIAL:
                driveToDetectInitial();
                break;
            case DRIVING_TO_DETECT_SECOND:
                driveToDetectSecond();
                break;
            case DEPOSITING_PURPLE_PIXEL:
                depositPurplePixel();
                break;
            case DRIVING_TO_BACKBOARD:
                driveToBackboard();
                break;
            case DEPOSITING_YELLOW_PIXEL:
                depositYellowPixel();
                break;
        }

        telemetry.addData("Status", state);
        telemetry.addData("Pose", robot.drive.getPoseEstimate());
        telemetry.addData("Hardware", robot);
    }

    public void driveToDetectInitial() {
        robot.drive.followTrajectorySequence(drivingToInitialDetectionTrajectory);

        if (robot.isTeamElementDetectedBow()) {
            teamElementLocation = TeamElementLocation.CENTER;
            state = State.DRIVING_TO_BACKBOARD;
        } else {
            state = State.DRIVING_TO_DETECT_SECOND;
        }
    }

    public void driveToDetectSecond() {
        drivingToSecondDetectionTrajectory = robot.drive.trajectorySequenceBuilder(drivingToInitialDetectionTrajectory.end()).lineTo(new Vector2d(23.77, -43.88)).build();
        robot.drive.followTrajectorySequence(drivingToSecondDetectionTrajectory);

        if (robot.isTeamElementDetectedBow()) {
            teamElementLocation = TeamElementLocation.RIGHT;
        } else {
            teamElementLocation = TeamElementLocation.LEFT;
        }

        state = State.DEPOSITING_PURPLE_PIXEL;
    }

    public void depositPurplePixel() {
        if (teamElementLocation.isDeferred()) {
            drivingToLastResortTrajectory = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate()).splineTo(new Vector2d(3.83, -34.00), Math.toRadians(154.82)).build();
            robot.drive.followTrajectorySequence(drivingToLastResortTrajectory);
        }

        // TODO: Run the intake backwards to vomit out the purple pixel.
        robot.intakeMotor.setPower(-0.5);
        robot.betterSleep(1500);

        state = State.DRIVING_TO_BACKBOARD;
    }

    public void driveToBackboard() {
        if (drivingToBackboardTrajectory == null) {
            drivingToBackboardTrajectory = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate()).splineTo(new Vector2d(50.87, -36.89), Math.toRadians(9.69)).build();
        }

        robot.drive.followTrajectorySequence(drivingToBackboardTrajectory);
    }

    public void moveToYellowPixelDepositLocation() {
        TrajectorySequence trajectorySequence;

        switch (teamElementLocation) {
            default:
                telemetry.addLine("`teamElementLocation` is null, defaulting to Center delivery.");
            case CENTER:
                return;
            case LEFT:
                trajectorySequence = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate()).strafeLeft(5.0).build();
                break;
            case RIGHT:
                trajectorySequence = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate()).strafeRight(5.0).build();
                break;
        }

        robot.drive.followTrajectorySequence(trajectorySequence);
        teamElementLocation = TeamElementLocation.CENTER;
    }

    public void depositYellowPixel() {
        moveToYellowPixelDepositLocation();

        robot.passoverArmLeft.setPosition(HWC.passoverDeliveryPos);
        robot.wrist.setPosition(HWC.wristDeliveryPos);

        if (robot.passoverArmLeft.getPosition() == HWC.passoverDeliveryPos) {
            robot.toggleClaw('L');
        }
    }
}
