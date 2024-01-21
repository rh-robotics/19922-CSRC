package org.firstinspires.ftc.teamcode.auton;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.auton.enums.AutonState;
import org.firstinspires.ftc.teamcode.subsystems.HWC;

/**
 * Main Autonomous OpMode for the robot
 */
@Autonomous (name = "Sleepy")
public class sleepAuton extends OpMode {
    HWC robot;
    String color = "red";

    @Override
    public void init() {
        // ------ Telemetry ------ //
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        // ------ Initialize Robot Hardware ------ //
        robot = new HWC(hardwareMap, telemetry);

        // ------ Reset Servos ------ //
        robot.clawL.setPosition(1);
        robot.clawR.setPosition(0);

        // ------ Telemetry ------ //
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if (robot.currentGamepad1.a && !robot.previousGamepad1.a) {
            if (color.equals("red")) { color = "blue"; }
            else if (color.equals("blue")) { color = "red"; }
        }

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Color", color);
        telemetry.update();
    }

    @Override
    public void start() {
        robot.betterSleep(10000);

        if(color.equals("red")) {
            robot.leftFront.setPower(-2);
            robot.leftRear.setPower(2);
            robot.rightFront.setPower(-2);
            robot.rightRear.setPower(2);
            robot.betterSleep(1500);
            robot.leftFront.setPower(0);
            robot.leftRear.setPower(0);
            robot.rightFront.setPower(0);
            robot.rightRear.setPower(0);
        } else if (color.equals("blue")) {
            robot.leftFront.setPower(2);
            robot.leftRear.setPower(-2);
            robot.rightFront.setPower(2);
            robot.rightRear.setPower(-2);
            robot.betterSleep(1500);
            robot.leftFront.setPower(0);
            robot.leftRear.setPower(0);
            robot.rightFront.setPower(0);
            robot.rightRear.setPower(0);
        }
    }

    @Override
    public void loop() {

    }
}
