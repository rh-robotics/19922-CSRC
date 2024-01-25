package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.HWC;
import org.firstinspires.ftc.teamcode.teleop.enums.MultiplierSelection;
import org.firstinspires.ftc.teamcode.teleop.enums.TeleOpState;

/**
 * TeleOp OpMode for simply driving with strafing wheels
 */
@TeleOp(name = "Pulley Test", group = "Primary OpModes")
public class pulleyTest extends OpMode {
public DcMotorEx rightPulley, leftPulley;

    @Override
    public void init() {
        // ------ Telemetry ------ //
        telemetry.addData("Status", "Initializing");
        telemetry.update();
        rightPulley = hardwareMap.get(DcMotorEx.class, "pulleyR");
        leftPulley = hardwareMap.get(DcMotorEx.class, "pulleyL");
        // ------ Initialize Robot Hardware ------ //
        leftPulley.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightPulley.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftPulley.setDirection(DcMotorSimple.Direction.REVERSE);
        rightPulley.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftPulley.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {


        ;
    }

    @Override
    public void loop() {
        rightPulley.setPower(gamepad1.right_stick_y);
        leftPulley.setPower(gamepad1.right_stick_y);
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftPulley.getPower(), rightPulley.getPower());
        telemetry.update();
    }
}