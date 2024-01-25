package org.firstinspires.ftc.teamcode.auton;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auton.enums.AutonState;
import org.firstinspires.ftc.teamcode.subsystems.HWC;

/**
 * Main Autonomous OpMode for the robot
 */
@Autonomous (name = "Odo Right")
public class OdometryAutonomousRight extends OpMode {
    HWC robot;
    AutonState state = AutonState.SCANNING_MIDDLE;
    boolean testingMode = false;

    @Override
    public void init() {
        // ------ Telemetry ------ //
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        // ------ Initialize Robot Hardware ------ //
        robot = new HWC(hardwareMap, telemetry);

        // ------ Initialize TFOD ------ //
        robot.initTFOD("fpaVision.tflite");

        // ------ Reset Servos ------ //
        robot.clawL.setPosition(1);
        robot.clawR.setPosition(0);

        // ------ Telemetry ------ //
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {

    }

    int tapeDistSide = 6500;
    int tapeDistCent = 8740;
    int strafeDist = 2000;
    int deliveryTime = 3000;
    int strafeShort = 1000;

    @Override
    public void loop() {
        if(robot.cv() == 1){
            telemetry.addData("Pos", "1");
            robot.odoStrafeLeft(strafeShort);
            robot.odoDrive(tapeDistSide);
            robot.sleepDeliver(deliveryTime);}

        else if (robot.cv() == 2){
            telemetry.addData("Pos", "2");
            robot.odoDrive(tapeDistCent);
            robot.sleepDeliver(deliveryTime);
        }
        else if (robot.cv() == 3){
            telemetry.addData("Pos", "3");
            robot.odoStrafeRight(strafeDist);
            robot.odoDrive(tapeDistSide);
            robot.sleepDeliver(deliveryTime);}

        else{
            robot.odoStrafeRight(strafeDist);
            if (robot.cv() != 0){
                robot.odoDrive(tapeDistSide);
                robot.sleepDeliver(deliveryTime);
            }
            else{
                robot.odoStrafeLeft(strafeDist+strafeShort);
                robot.odoDrive(tapeDistSide);
                robot.sleepDeliver(deliveryTime);
            }
        }


        telemetry.addData("Status", "Running");
        telemetry.addData("State", state);
    }
}