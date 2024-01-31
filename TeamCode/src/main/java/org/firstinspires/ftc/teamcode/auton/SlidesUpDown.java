package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.HWC;

@Autonomous
public class SlidesUpDown extends LinearOpMode {
    HWC robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new HWC(hardwareMap, telemetry);

        waitForStart();
        while (opModeIsActive()) {

            robot.powerSlides(1);
            robot.betterSleep(1750);
            robot.powerSlides(-1);
            robot.betterSleep(1750);
        }
    }
}
