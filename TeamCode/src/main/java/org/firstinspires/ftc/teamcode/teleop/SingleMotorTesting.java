package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * TeleOp OpMode for simply driving with strafing wheels
 */
@TeleOp(name = "Pulley Test", group = "Primary OpModes")
public class SingleMotorTesting extends OpMode {
public DcMotorEx motor;

    @Override
    public void init() {
        // ------ Telemetry ------ //
        telemetry.addData("Status", "Initializing");
        telemetry.update();
        motor = hardwareMap.get(DcMotorEx.class, "motor");

        // ------ Initialize Robot Hardware ------ //
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {}

    @Override
    public void loop() {
        motor.setPower(gamepad1.right_stick_y);
        telemetry.addData("Motor Power", motor.getPower());
        telemetry.addData("Motor Position", motor.getCurrentPosition());
        telemetry.addData("Motor Velocity", motor.getVelocity());
        telemetry.update();
    }
}