package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.HWC;

@Autonomous(name = "AutonomousV1")
public class AutonomousV1 extends OpMode {
    private enum State {
        DRIVING_TO_DETECT_INITIAL, DRIVING_TO_DETECT_SECOND, DEPOSITING_PURPLE_PIXEL, DRIVING_TO_BACKBOARD, DEPOSITING_YELLOW_PIXEL, STOP
    }

    private enum TeamElementLocation {
        CENTER, LEFT, RIGHT;

        public boolean isDeferred() {
            return this == RIGHT;
        }
    }

    // ------ Variables ------ //
    private TeamElementLocation teamElementLocation;
    private State[] toSelect = new State[]{State.DRIVING_TO_DETECT_INITIAL, State.DRIVING_TO_DETECT_SECOND, State.DEPOSITING_PURPLE_PIXEL, State.DRIVING_TO_BACKBOARD, State.DEPOSITING_YELLOW_PIXEL, State.STOP};
    private State selected;
    private State state = State.DRIVING_TO_DETECT_INITIAL;
    private HWC robot;
    private boolean novel = true;
    private boolean go = false;
    private int toSelectIndex = 0;

    // ------ Trajectories ------ //
    public Trajectory activeTrajectory;
    private Trajectory drivingToInitialDetectionTrajectory;
    private Trajectory drivingToSecondDetectionTrajectory;
    private Trajectory drivingToLastResortTrajectory;
    private Trajectory drivingToBackboardTrajectory;
    private Pose2d startPose = new Pose2d(11.84, -60.75, Math.toRadians(90.00));

    @Override
    public void init() {
        // ------ Telemetry ------ //
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        // ------ Initialize Robot Hardware ------ //
        robot = new HWC(hardwareMap, telemetry);

        // ------ Get Trajectories All Good ------ //
        robot.drive.setPoseEstimate(startPose);
        telemetry.addData("Pose", robot.drive.getPoseEstimate()); // REMOVEME
        drivingToInitialDetectionTrajectory = robot.drive.trajectoryBuilder(startPose).lineTo(new Vector2d(11.67, -44.39)).build();

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
        // ------ GamePad Updates ------ //
        robot.previousGamepad1.copy(robot.currentGamepad1);
        robot.previousGamepad2.copy(robot.currentGamepad2);
        robot.currentGamepad1.copy(gamepad1);
        robot.currentGamepad2.copy(gamepad2);


        State prevState = state;

        if (!go) {
            telemetry.addData("Selected State", state + " (" + toSelectIndex + ")");

            if (robot.currentGamepad1.a && !robot.previousGamepad1.a) {
                state = toSelect[toSelectIndex++];
            }

            if (robot.currentGamepad1.b && !robot.previousGamepad1.b) {
                selected = state;
                go = true;
            }

            return;
        }

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
            case STOP:
                stop();
        }

        robot.drive.update();
        novel = state != prevState;

        telemetry.addData("Status", state);
        telemetry.addData("Pose", robot.drive.getPoseEstimate());
        telemetry.addData("Team Element Location", teamElementLocation);
        telemetry.addData("Novelity", novel);
        telemetry.addData("Hardware", robot);

        if (activeTrajectory != null) {
            telemetry.addData("Trajectory", activeTrajectory);
        }

        if (teamElementLocation != null) {
            telemetry.addData("Deferred Pixel", teamElementLocation.isDeferred());
        }

        if (state != selected) {
            stop();
        }

        telemetry.update();
    }

    public void driveToDetectInitial() {
        if (followTrajectory(drivingToInitialDetectionTrajectory)) return;

        if (robot.isTeamElementDetectedBow()) {
            teamElementLocation = TeamElementLocation.CENTER;
            state = State.DRIVING_TO_BACKBOARD;
        } else {
            state = State.DRIVING_TO_DETECT_SECOND;
        }
    }

    public void driveToDetectSecond() {
        drivingToSecondDetectionTrajectory = robot.drive.trajectoryBuilder(drivingToInitialDetectionTrajectory.end()).lineTo(new Vector2d(23.77, -43.88)).build();
        if (followTrajectory(drivingToSecondDetectionTrajectory)) return;

        if (robot.isTeamElementDetectedBow()) {
            teamElementLocation = TeamElementLocation.RIGHT;
        } else {
            teamElementLocation = TeamElementLocation.LEFT;
        }

        state = State.DEPOSITING_PURPLE_PIXEL;
    }

    public void depositPurplePixel() {
        if (teamElementLocation.isDeferred()) {
            drivingToLastResortTrajectory = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate()).splineTo(new Vector2d(3.83, -34.00), Math.toRadians(154.82)).build();
            if (followTrajectory(drivingToLastResortTrajectory)) return;
        }

        robot.intakeMotor.setPower(-0.5);
        robot.betterSleep(1500);
        robot.intakeMotor.setPower(0);

        state = State.DRIVING_TO_BACKBOARD;
    }

    public void driveToBackboard() {
        if (drivingToBackboardTrajectory == null) {
            drivingToBackboardTrajectory = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate()).splineTo(new Vector2d(50.87, -36.89), Math.toRadians(9.69)).build();
        }

        if (followTrajectory(drivingToBackboardTrajectory)) return;

        state = State.DEPOSITING_YELLOW_PIXEL;
    }

    public void moveToYellowPixelDepositLocation() {
        Trajectory trajectorySequence;

        switch (teamElementLocation) {
            default:
                telemetry.addLine("`teamElementLocation` is null, defaulting to Center delivery.");
            case CENTER:
                return;
            case LEFT:
                trajectorySequence = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate()).strafeLeft(5.0).build();
                break;
            case RIGHT:
                trajectorySequence = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate()).strafeRight(5.0).build();
                break;
        }

        if (followTrajectory(trajectorySequence)) return;
        teamElementLocation = TeamElementLocation.CENTER;
    }

    public void depositYellowPixel() {
        moveToYellowPixelDepositLocation();

        robot.passoverArmLeft.setPosition(HWC.passoverDeliveryPos);
        robot.wrist.setPosition(HWC.wristDeliveryPos);

        if (robot.passoverArmLeft.getPosition() == HWC.passoverDeliveryPos) {
            robot.toggleClaw('L');
            state = State.STOP;
        }
    }

    public boolean followTrajectory(Trajectory trajectory) {
        if (novel) {
            robot.drive.followTrajectoryAsync(trajectory);
            activeTrajectory = trajectory;
            novel = false;
            return true;
        }

        return robot.drive.isBusy();
    }
}
