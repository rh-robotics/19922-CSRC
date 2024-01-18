package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.teleop.enums.MultiplierSelection;

/**
 * Enum representing which speed to change in init_loop()
 */


/**
 * TeleOp OpMode for simply driving with strafing wheels
 */
@TeleOp(name = "Drive Only", group = "Testing")
public class DriveOnly extends OpMode {
    public DcMotorEx leftFront, rightFront, leftRear, rightRear;
    double turnSpeed = 0.6; // Speed multiplier for turning
    double driveSpeed = 1; // Speed multiplier for driving
    double strafeSpeed = 1; // Speed multiplier for strafing
    MultiplierSelection selection; // String for selecting which speed to change

    // init() Runs ONCE after the driver hits initialize
    @Override
    public void init() {

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

        // Tell the driver the Op is initializing
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        // Initialize the robot hardware

        // Tell the driver the robot is ready
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    // init_loop() - Runs continuously until the driver hits play
    @Override
    public void init_loop() {
        // Select which speed to change


        // Change the speed multiplier for selection


        // Tell driver the commands
        telemetry.addData("Press A to start changing turn speed", "");
        telemetry.addData("Press B to start changing drive speed", "");
        telemetry.addData("Press X to start changing strafe speed", "");

        // Divider Line
        telemetry.addLine();

        // Display the current modifiers
        telemetry.addData("Modifying", selection);
        telemetry.addData("Turn Speed", turnSpeed);
        telemetry.addData("Drive Speed", driveSpeed);
        telemetry.addData("Strafe Speed", strafeSpeed);

        // Update telemetry
        telemetry.update();
    }

    // Start() - Runs ONCE when the driver presses play
    @Override
    public void start() {
    }

    // loop() - Runs continuously while the OpMode is active
    @Override
    public void loop() {

        double leftFPower;
        double rightFPower;
        double leftBPower;
        double rightBPower;
        double drive = -gamepad1.left_stick_x * driveSpeed;
        double turn = gamepad1.left_stick_y * turnSpeed;
        double strafe = -gamepad1.right_stick_x * strafeSpeed;

        // --------------- Calculate drive power --------------- //
        if (drive != 0 || turn != 0) {
            leftFPower = Range.clip(drive + turn, -1.0, 1.0);
            rightFPower = Range.clip(drive - turn, -1.0, 1.0);
            leftBPower = Range.clip(drive + turn, -1.0, 1.0);
            rightBPower = Range.clip(drive - turn, -1.0, 1.0);
        } else if (strafe != 0) {
            // Strafing
            leftFPower = -strafe;
            rightFPower = strafe;
            leftBPower = strafe;
            rightBPower = -strafe;
        } else {
            leftFPower = 0;
            rightFPower = 0;
            leftBPower = 0;
            rightBPower = 0;
        }
        rightFront.setPower(rightFPower);
        leftFront.setPower(leftFPower);
        rightRear.setPower(rightBPower);
        leftRear.setPower(leftBPower);


        // --------------- Telemetry Updates --------------- //
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFPower, rightFPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBPower, rightBPower);
        telemetry.update();
    }


}