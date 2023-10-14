package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.HWC;
import org.firstinspires.ftc.teamcode.subsystems.roadrunner.drive.SampleMecanumDrive;

@Autonomous
@Disabled
public class SimpleRRAuton extends LinearOpMode {
    HWC robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new HWC(hardwareMap, telemetry);

        Trajectory forward = robot.drive.trajectoryBuilder(new Pose2d(0, 0))
                .forward(5)
                .build();

        Trajectory strafeLeft = robot.drive.trajectoryBuilder(forward.end())
                .strafeLeft(5)
                .build();

        Trajectory backwards = robot.drive.trajectoryBuilder(strafeLeft.end())
                .back(5)
                .build();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // Set status to initialized
        telemetry.addData("Status", "Initialized");

        // Update telemetry to show which trajectory is running & run trajectory
        telemetry.addData("Trajectory", "Forward");
        telemetry.update();
        robot.drive.followTrajectory(forward);

        // Update telemetry to show which trajectory is running & run trajectory
        telemetry.addData("Trajectory", "Backward");
        telemetry.update();
        robot.drive.followTrajectory(backwards);

        // Update telemetry to show which trajectory is running & run trajectory
        telemetry.addData("Trajectory", "StrafeLeft");
        telemetry.update();
        robot.drive.followTrajectory(strafeLeft); // Run the strafeLeft trajectory

    }
}