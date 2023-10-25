package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.HWC;

/**
 * TeleOp OpMode for simply driving with strofing wheels
 */
@TeleOp(name = "QCR2 TeleOp", group = "Iterative OpMode")
public class TeleopMode extends OpMode {
    private final ElapsedTime time = new ElapsedTime();
    HWC robot; // Declare the object for HWC, will allow us to access all the motors declared there!

    //Speed Variables
    double tSpeed = 0.6;
    double dSpeed = 0.8;
    double sSpeed = 0.8;


    // init() Runs ONCE after the driver hits initialize
    @Override
    public void init() {
        // Tell the driver the Op is initializing
        telemetry.addData("Status", "Initializing");

        // Do all init stuff
        // TODO: ADD INITS THAT YOU NEED

        robot = new HWC(hardwareMap, telemetry);

        // Tell the driver the robot is ready
        telemetry.addData("Status", "Initialized");
    }

    // init_loop() - Runs continuously until the driver hits play
    @Override
    public void init_loop() {
        if (gamepad1.a == true) {

            if (gamepad1.dpad_down == true) {
                tSpeed -= 1;
            } else if (gamepad1.dpad_up == true) {
                tSpeed += 1;
            } else if (gamepad1.b == true) {

                if (gamepad1.dpad_down == true) {
                    dSpeed -= 1;
                } else if (gamepad1.dpad_up) {
                    dSpeed += 1;
                } else if (gamepad1.x == true) {
                    if (gamepad1.dpad_down == true) {
                        sSpeed -= 1;
                    } else if (gamepad1.dpad_up) {
                        sSpeed += 1;
                    }
                }
            }
        }
    }

    // Start() - Runs ONCE when the driver presses play
    @Override
    public void start() {
        time.reset();
    }

    // loop() - Runs continuously while the OpMode is active
    @Override
    public void loop() {
        double leftFPower;
        double rightFPower;
        double leftBPower;
        double rightBPower;
        double drive = -gamepad1.left_stick_x * dSpeed;
        double turn = gamepad1.left_stick_y * tSpeed;
        double strafe = -gamepad1.right_stick_x * sSpeed;

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

        // Set power to values calculated above
        robot.leftFront.setPower(leftFPower);
        robot.leftRear.setPower(leftBPower);
        robot.rightFront.setPower(rightFPower);
        robot.rightRear.setPower(rightBPower);

        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFPower, rightFPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBPower, rightBPower);
    }
}