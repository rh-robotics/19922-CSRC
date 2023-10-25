package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.HWC;

/**
 * TeleOp OpMode for simply driving with strafing wheels
 */
@TeleOp(name = "TeleOp", group = "Iterative OpMode")
public class TeleOpMode extends OpMode {
    HWC robot; // Declare the object for HWC, will allow us to access all the motors declared there!
    double turnSpeed = 0.6; // Speed multiplier for turning
    double driveSpeed = 0.8; // Speed multiplier for driving
    double strafeSpeed = 0.8; // Speed multiplier for strafing
    MultiplierSelection selection; // String for selecting which speed to change

    // init() Runs ONCE after the driver hits initialize
    @Override
    public void init() {
        // Tell the driver the Op is initializing
        telemetry.addData("Status", "Initializing");
        telemetry.update();


        robot = new HWC(hardwareMap, telemetry);


        // Tell the driver the robot is ready
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    // init_loop() - Runs continuously until the driver hits play
    @Override
    public void init_loop() {
        // Select which speed to change
        if(gamepad1.a) { selection = MultiplierSelection.TURN_SPEED; }
        else if(gamepad1.b) { selection = MultiplierSelection.DRIVE_SPEED; }
        else if(gamepad1.x) { selection = MultiplierSelection.STRAFE_SPEED; }

        // Change the speed multiplier for selection
        switch(selection) {
            case TURN_SPEED:
                if(gamepad1.dpad_up) { turnSpeed += 0.1; }
                else if(gamepad1.dpad_down) { turnSpeed -= 0.1; }
                break;
            case DRIVE_SPEED:
                if(gamepad1.dpad_up) { driveSpeed += 0.1; }
                else if(gamepad1.dpad_down) { driveSpeed -= 0.1; }
                break;
            case STRAFE_SPEED:
                if(gamepad1.dpad_up) { strafeSpeed += 0.1; }
                else if(gamepad1.dpad_down) { strafeSpeed -= 0.1; }
                break;
        }

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
        robot.time.reset();
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

        // Calculate drive power
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

        // Run Intake
       if (gamepad1.right_trigger != 0){
           robot.runIntake(gamepad1.right_trigger);
       }

        // Set power to values calculated above
        robot.leftFront.setPower(leftFPower);
        robot.leftRear.setPower(leftBPower);
        robot.rightFront.setPower(rightFPower);
        robot.rightRear.setPower(rightBPower);

        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFPower, rightFPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBPower, rightBPower);
        telemetry.update();
    }
}

/**
 * Enum representing which speed to change in init_loop()
 */
enum MultiplierSelection {
    TURN_SPEED,
    DRIVE_SPEED,
    STRAFE_SPEED
}