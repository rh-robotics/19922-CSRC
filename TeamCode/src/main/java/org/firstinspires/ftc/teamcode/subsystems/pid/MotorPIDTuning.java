package org.firstinspires.ftc.teamcode.subsystems.pid;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.HWC;

@Config
@TeleOp(name = "Motor PID Tuning", group = "Testing")
public class MotorPIDTuning extends OpMode {
    // ------ Declare PID Variables ------ //
    public static double p = 0, i = 0, d = 0, f = 0;
    public static int target = 0;
    private double motorPos = 0;
    private double pid = 0;
    private double ff = 0;

    // ------ Declare Others ------ //
    private PIDController controller;
    private HWC kit;
    private DcMotorEx slideLeft;
    //private Motor motorSelection = Motor.SLIDE_L;
    private DcMotorEx slideRight;
    private final double TICKS_IN_DEGREES = 751.8 / 360;

    @Override
    public void init() {
        // ------ Initialize Hardware ------ //
        kit = new HWC(hardwareMap, telemetry, false);

        slideLeft = kit.leftPulley;
        slideRight = kit.rightPulley;

        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // ------ Initialize PID Controller ------ //
        controller = new PIDController(p, i, d);

        // ------ Make telemetry FTC Dashboard Compatible ------ //
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        slideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        // ------ Update Gamepads ------ //
        kit.previousGamepad1.copy(kit.currentGamepad1);
        kit.previousGamepad2.copy(kit.currentGamepad2);
        kit.currentGamepad1.copy(gamepad1);
        kit.currentGamepad2.copy(gamepad2);

        // ------ Set PID ------ //
        controller.setPID(p, i, d);

        // ------ Get Motor Position ------ //
        motorPos = (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition()) / 2.0;

        // ------ Calculate PID ------ //
        pid = controller.calculate(motorPos, target);

        // ------ Calculate Feed Forward ------ //
        ff = Math.cos(Math.toRadians(target / TICKS_IN_DEGREES)) * f;

        // ------ Set Motor Power ------ //
        slideLeft.setPower(pid + ff);
        slideRight.setPower(pid + ff);

        // ------ Telemetry ------ //
        telemetry.addData("Motor Position", motorPos);
        telemetry.addData("Motor Target", target);
        telemetry.addData("Motor Power", slideLeft.getPower());
        telemetry.addData("P", p);
        telemetry.addData("I", i);
        telemetry.addData("D", d);
        telemetry.addData("F", f);
        telemetry.addData("Calculated PID", pid);
        telemetry.addData("Calculated FF", ff);
        telemetry.update();
    }
}