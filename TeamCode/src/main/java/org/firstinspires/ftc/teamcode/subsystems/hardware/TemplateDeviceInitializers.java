package org.firstinspires.ftc.teamcode.subsystems.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;

/** A collection of template device initializers. */
public class TemplateDeviceInitializers {
    /** Initializes a DcMotorEx to be reversed, brake on zero-power, and to use encoders.
     * @param dev The device to initialize.
     */
    public static void motorReverseBrakeEncoder(HardwareDevice dev) {
        DcMotorEx motor = (DcMotorEx) dev;
        motor.setDirection(DcMotorEx.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    /** Initializes a DcMotorEx to go forward, brake on zero-power, and to use encoders.
     * @param dev The device to initialize.
     */
    public static void motorForwardBrakeEncoder(HardwareDevice dev) {
        DcMotorEx motor = (DcMotorEx) dev;
        motor.setDirection(DcMotorEx.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
}
