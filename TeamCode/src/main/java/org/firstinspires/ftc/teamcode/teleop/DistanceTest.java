package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.HWC;

/**
 * TeleOp OpMode for simply driving with strafing wheels
 */
@TeleOp(name = "Distance test", group = "Primary OpModes")
public class DistanceTest extends OpMode {

    // ------ Declare Others ------ //
    private HWC robot;
    private double frontLeftPower;
    private double frontRightPower;
    private double backLeftPower;
    private double backRightPower;
    private boolean aligning = false;

    @Override
    public void init() {
        // ------ Telemetry ------ //
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        // ------ Initialize Robot Hardware ------ //
        robot = new HWC(hardwareMap, telemetry, false);

        // ------ Telemetry ------ //
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        robot.time.reset();
    }

    @Override
    public void loop() {
        // ------ GamePad Updates ------ //
        robot.previousGamepad1.copy(robot.currentGamepad1);
        robot.previousGamepad2.copy(robot.currentGamepad2);
        robot.currentGamepad1.copy(gamepad1);
        robot.currentGamepad2.copy(gamepad2);


        if (gamepad1.dpad_left) {
           aligning = true;
        }
        if (aligning){
            alignWithBackboard(19, 2);
        }
        if (gamepad1.dpad_right) {
            aligning = false;
            frontLeftPower = 0;
            backLeftPower = 0;
            frontRightPower = 0;
            backRightPower = 0;
            telemetry.addData("Left Distance", robot.distLeft.getDistance(DistanceUnit.CM));
            telemetry.addData("Right Distance", robot.distRight.getDistance(DistanceUnit.CM));
        }

        robot.leftFront.setPower(frontLeftPower);
        robot.leftRear.setPower(backLeftPower);
        robot.rightFront.setPower(frontRightPower);
        robot.rightRear.setPower(backRightPower);
    }

    // ------ Function to Align With Backboard ------ //
    private void alignWithBackboard(int dist, int tolerance) {
        int distPlus = dist + tolerance;
        int distMinus = dist - tolerance;

        if (robot.distRight.getDistance(DistanceUnit.CM) >= distMinus && robot.distRight.getDistance(DistanceUnit.CM )<= distMinus && robot.distLeft.getDistance(DistanceUnit.CM) >= distMinus && robot.distLeft.getDistance(DistanceUnit.CM )<= distMinus){

        }


        if (distPlus >= robot.distRight.getDistance(DistanceUnit.CM) && robot.distRight.getDistance(DistanceUnit.CM) >= distMinus) {
            frontRightPower = 0;
            backRightPower = 0;

            if (distPlus >= robot.distLeft.getDistance(DistanceUnit.CM) && robot.distLeft.getDistance(DistanceUnit.CM) >= distMinus) {
                frontLeftPower = 0;
                backLeftPower = 0;
                aligning = false;
            } else if (robot.distRight.getDistance(DistanceUnit.CM) < distMinus) {
                frontLeftPower = 0.3;
                backLeftPower = 0.3;
            }
        } else if (distPlus >= robot.distLeft.getDistance(DistanceUnit.CM) && robot.distLeft.getDistance(DistanceUnit.CM) >= distMinus) {
            frontLeftPower = 0;
            backLeftPower = 0;

            if (distPlus >= robot.distRight.getDistance(DistanceUnit.CM) && robot.distRight.getDistance(DistanceUnit.CM) >= distMinus) {
                frontRightPower = 0;
                backRightPower = 0;
                aligning = false;
            } else if (robot.distRight.getDistance(DistanceUnit.CM) > distPlus) {
                frontRightPower = -0.3;
                backRightPower = -0.3;
            }
        } else {
            frontLeftPower = 0.2;
            frontRightPower = -0.2;
            backLeftPower = 0.2;
            backRightPower = -0.2;
        }
        if (distPlus >= robot.distRight.getDistance(DistanceUnit.CM) && robot.distRight.getDistance(DistanceUnit.CM) >= distMinus && distPlus >= robot.distLeft.getDistance(DistanceUnit.CM) && robot.distLeft.getDistance(DistanceUnit.CM) >= distMinus) {
            frontLeftPower = 0;
            backLeftPower = 0;
            frontRightPower = 0;
            backRightPower = 0;
            aligning = false;
        } else {
            if (distMinus > robot.distRight.getDistance(DistanceUnit.CM) && distMinus > robot.distLeft.getDistance(DistanceUnit.CM)) {
                frontLeftPower = -0.2;
                backLeftPower = -0.2;
                frontRightPower = 0.2;
                backRightPower = 0.2;
            } else if (distMinus > robot.distRight.getDistance(DistanceUnit.CM)) {
                frontLeftPower = 0;
                backLeftPower = 0;
                frontRightPower = 0.3;
                backRightPower = 0.3;
            } else if (distMinus > robot.distLeft.getDistance(DistanceUnit.CM)) {

                frontLeftPower = -0.3;
                backLeftPower = -0.3;
                frontRightPower = 0;
                backRightPower = 0;
            }

        }
        telemetry.addData("Left Distance", robot.distLeft.getDistance(DistanceUnit.CM));
        telemetry.addData("Right Distance", robot.distRight.getDistance(DistanceUnit.CM));
        telemetry.addData("Right Power", robot.rightRear.getPower());
        telemetry.addData("Left Power", robot.leftRear.getPower());
        telemetry.update();


    }

}