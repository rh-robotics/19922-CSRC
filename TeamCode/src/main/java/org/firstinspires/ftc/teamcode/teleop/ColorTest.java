package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.HWC;

/**
 * TeleOp OpMode for simply driving with strafing wheels
 */
@TeleOp(name = "Color Sensor Test", group = "Testing")
public class ColorTest extends OpMode {
    HWC robot;

    @Override
    public void init() {
        // ------ Telemetry ------ //
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        // ------ Initialize Robot Hardware ------ //
        robot = new HWC(hardwareMap, telemetry, false);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            telemetry.addData("Color", robot.colorLeft.argb());
        } else if (gamepad1.b) {

            telemetry.addData("Color", robot.colorRight.argb());
        } else if (gamepad1.y) {
            telemetry.addData("Color", "Not Detecting");
        }

        telemetry.update();
    }
}