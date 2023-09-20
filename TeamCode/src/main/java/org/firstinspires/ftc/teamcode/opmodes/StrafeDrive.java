package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.hardware.HardwareSchema;
import org.firstinspires.ftc.teamcode.subsystems.hardware.TemplateDeviceInitializers;

/** A basic strafe drive opmode. */
@TeleOp(name = "Basic Strafe Drive", group = "Iterative OpMode")
public class StrafeDrive extends OpMode {
    /**
     * Shove our robots physical parts in here.
     */
    private Hardware robot;

    /**
     * The time that has elapsed since the initialization of the hardware.
     */
    private final ElapsedTime time = new ElapsedTime();

    /** Runs ONCE after the driver hits initialize.
     * <p>
     * Sets up the robot physically, digitally, and mentally for the trials ahead.
     */
    @Override
    public void init() {
        // Tell the driver the Op is initializing.
        telemetry.addData("Status", "Initializing");

        // Create the initialization schema.
        HardwareSchema hardwareSchema = new HardwareSchema();
        hardwareSchema.introduce("leftFront", "leftFront", DcMotorEx.class,
                TemplateDeviceInitializers::motorReverseBrakeEncoder);
        hardwareSchema.introduce("leftRear", "leftRear", DcMotorEx.class,
                TemplateDeviceInitializers::motorReverseBrakeEncoder);
        hardwareSchema.introduce("rightFront", "rightFront", DcMotorEx.class,
                TemplateDeviceInitializers::motorForwardBrakeEncoder);
        hardwareSchema.introduce("rightRear", "rightRear", DcMotorEx.class,
                TemplateDeviceInitializers::motorForwardBrakeEncoder);

        // Do initialization and check that everything is all fine and dandy.
        robot = new Hardware(hardwareSchema, hardwareMap, telemetry);

        // Tell the driver the robot is ready.
        telemetry.addData("Status", "Initialized");
    }

    /**
     * Runs ONCE when the driver presses play. Very little initialization should be done here, if
     * any. Maybe resetting the elapsed time.
     */
    @Override
    public void start() {
        time.reset();
    }

    /**
     * Code that is called once per frame. Process input, send output to motors, etc.
     * Do it all here!
     */
    @Override
    public void loop() {
        double leftFPower;
        double rightFPower;
        double leftBPower;
        double rightBPower;
        double drive = -gamepad1.left_stick_y * 0.8;
        double turn = gamepad1.left_stick_x * 0.6;
        double strafe = -gamepad1.right_stick_x * 0.8;

        // Calculate drive power
        if (drive != 0 || turn != 0) {
            // Normal.
            leftFPower = Range.clip(drive + turn, -1.0, 1.0);
            rightFPower = Range.clip(drive - turn, -1.0, 1.0);
            leftBPower = Range.clip(drive + turn, -1.0, 1.0);
            rightBPower = Range.clip(drive - turn, -1.0, 1.0);
        } else if (strafe != 0) {
            // Strafing.
            leftFPower = -strafe;
            rightFPower = strafe;
            leftBPower = strafe;
            rightBPower = -strafe;
        } else {
            // Stalling.
            leftFPower = 0;
            rightFPower = 0;
            leftBPower = 0;
            rightBPower = 0;
        }

        // Set power to values calculated above.
        robot.getDevice("leftFront", DcMotorEx.class).setPower(leftFPower);
        robot.getDevice("leftRear", DcMotorEx.class).setPower(leftBPower);
        robot.getDevice("rightFront", DcMotorEx.class).setPower(rightFPower);
        robot.getDevice("rightRear", DcMotorEx.class).setPower(rightBPower);
    }
}