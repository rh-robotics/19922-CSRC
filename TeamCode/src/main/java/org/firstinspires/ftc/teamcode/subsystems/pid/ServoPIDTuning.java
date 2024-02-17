package org.firstinspires.ftc.teamcode.subsystems.pid;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.subsystems.HWC;

public class ServoPIDTuning extends OpMode {
    // ------ Motor Enum ------ //
    private enum Servo {
        PASSOVER_L, PASSOVER_R
    }

    // ------ Declare PID Variables ------ //
    public static double p = 0, i = 0, d = 0, f = 0;
    public static int target = 0;
    private int servoPos = 0;
    private double pid = 0;
    private double ff = 0;

    // ------ Declare Others ------ //
    private PIDController controller;
    private HWC kit;
    private CRServo servo;
    private final Servo[] servoSelectionList = new Servo[]{ Servo.PASSOVER_R, Servo.PASSOVER_L };
    private int servoSelectionIndex = 0;
    // TODO: Update with correct ticks amount of ticks for motor
    private final double TICKS_IN_DEGREES = 435.5 / 360; // Ticks Per Degree = Ticks Per Rotation / 360

    @Override
    public void init() {
        // ------ Initialize Hardware ------ //
        kit = new HWC(hardwareMap, telemetry);

        // ------ Initialize PID Controller ------ //
        controller = new PIDController(p, i, d);

        // ------ Make telemetry FTC Dashboard Compatible ------ //
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        // ------ Update Gamepads ------ //
        kit.previousGamepad1.copy(kit.currentGamepad1);
        kit.previousGamepad2.copy(kit.currentGamepad2);
        kit.currentGamepad1.copy(gamepad1);
        kit.currentGamepad2.copy(gamepad2);

        // ------ Update Motor Index ------ //
        if(kit.currentGamepad1.a && !kit.previousGamepad1.a) {
            servoSelectionIndex++;
            p = 0; i = 0; d = 0; f = 0;
        }

        // ------ Update Motor Value ------ //
        switch(servoSelectionList[servoSelectionIndex]) {
            case PASSOVER_L:
                servo = kit.passoverArmLeft;
                break;
            case PASSOVER_R:
                servo = kit.rightPulley;
                break;
        }

        // ------ Set PID ------ //
        controller.setPID(p, i ,d);

        // ------ Get Motor Position ------ //
        servoPos = analogInput.getVoltage() / 3.3 * 360;

        // ------ Calculate PID ------ //
        pid = controller.calculate(servoPos, target);

        // ------ Calculate Feed Forward ------ //
        ff = Math.cos(Math.toRadians(target / TICKS_IN_DEGREES)) * f;

        // ------ Set Motor Power ------ //
        servo.setPower(pid + ff);

        // ------ Telemetry ------ //
        telemetry.addData("Motor", servoSelectionList[servoSelectionIndex]);
        telemetry.addData("Motor Position", servoPos);
        telemetry.addData("Motor Target", target);
        telemetry.addData("Motor Power", servo.getPower());
        telemetry.addData("P", p);
        telemetry.addData("I", i);
        telemetry.addData("D", d);
        telemetry.addData("F", f);
        telemetry.addData("Calculated PID", pid);
        telemetry.addData("Calculated FF", ff);
        telemetry.update();
    }
}
