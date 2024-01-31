package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.subsystems.HWC;

import org.firstinspires.ftc.teamcode.subsystems.HWC;
import org.firstinspires.ftc.teamcode.teleop.enums.MultiplierSelection;
import org.firstinspires.ftc.teamcode.teleop.enums.TeleOpState;

/**
 * TeleOp OpMode for simply driving with strafing wheels
 */
@TeleOp(name = "Color Sensor Test", group = "Primary OpModes")
public class ColorTest extends OpMode {
    HWC robot;
    @Override
    public void init() {
        robot = new HWC(hardwareMap, telemetry);
        // ------ Telemetry ------ //
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        // ------ Initialize Robot Hardware ------ //


    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        if (gamepad1.a){
            telemetry.addData("Color",robot.colorLeft.argb());
        }
        else if (gamepad1.b){

            telemetry.addData("Color", robot.colorRight.argb());
        }
        else if (gamepad1.y){
            telemetry.addData("Color", "Not Detecting");
        }

         telemetry.update();
    }
}