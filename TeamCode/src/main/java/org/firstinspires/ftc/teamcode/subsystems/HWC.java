package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.roadrunner.drive.SampleMecanumDrive;

import java.util.Objects;

/**
 * Stores and Declares all hardware devices &amp; related methods
 */
public class HWC {
    // ------ Declare Motors ------ //
    public DcMotorEx leftFront, rightFront, leftRear, rightRear, rightPulley, leftPulley, intakeMotor;

    // ------ Declare Servos ------ //
    public Servo intakeArm, wrist, clawR, clawL, passoverArmLeft, passoverArmRight;

    // ------ Declare Sensors ------ //
    public ColorSensor colorLeft, colorRight;

    // ------ Declare Gamepads ------ //
    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad currentGamepad2 = new Gamepad();
    public Gamepad previousGamepad1 = new Gamepad();
    public Gamepad previousGamepad2 = new Gamepad();

    // ------ Declare Webcam ------ //
    public WebcamName webcam;

    // ------ Declare Roadrunner Drive ------ //
    public SampleMecanumDrive drive;

    // ------ ElapsedTime Variable ------ //
    public ElapsedTime time = new ElapsedTime();

    // ------ Telemetry ------ //
    Telemetry telemetry;

    // ------ Position Variables ------ //
    //TODO: UPDATE WITH REAL NUMBERS ONCE TESTED

    int intakePos = 5;
    int armDeliveryPos = 6;
    int armRetractedPos = 0;
    double wristDeliveryPos = 20;
    double wristIntakePos = 0;

    /**
     * Constructor for HWC, declares all hardware components
     *
     * @param hardwareMap HardwareMap - Used to retrieve hardware devices
     * @param telemetry   Telemetry - Used to add telemetry to driver hub
     */
    public HWC(@NonNull HardwareMap hardwareMap, Telemetry telemetry) {
        // ------ Telemetry ------ //
        this.telemetry = telemetry;

        // ------ Declare RR Drivetrain ------ //
        drive = new SampleMecanumDrive(hardwareMap);

        // ------ Retrieve Drive Motors ------ //
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

        // ------ Retrieve Other Motors ------ //
        rightPulley = hardwareMap.get(DcMotorEx.class, "pulleyR");
        leftPulley = hardwareMap.get(DcMotorEx.class, "pulleyL");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        // ------ Retrieve Servos ------ //
        intakeArm = hardwareMap.get(Servo.class, "intakeArm");
        clawL = hardwareMap.get(Servo.class, "clawL");
        clawR = hardwareMap.get(Servo.class, "clawR");
        wrist = hardwareMap.get(Servo.class, "wrist");
        passoverArmLeft = hardwareMap.get(Servo.class, "passoverArmLeft");
        passoverArmRight = hardwareMap.get(Servo.class, "passoverArmRight");

        // ------ Retrieve Sensors ------ //
        webcam = hardwareMap.get(WebcamName.class, "webcam");
        colorLeft = hardwareMap.get(ColorSensor.class, "colorL");
        colorRight = hardwareMap.get(ColorSensor.class, "colorR");

        // ------ Set Motor Directions ------ //
        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftRear.setDirection(DcMotorEx.Direction.FORWARD);
        rightRear.setDirection(DcMotorEx.Direction.FORWARD);
        leftPulley.setDirection(DcMotorEx.Direction.REVERSE);

        // ------ Set Motor Brake Modes ------ //
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightPulley.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftPulley.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ------ Set Motor Modes ------ //
        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    // ------ Function to Run Intake ------ //
    // Runs intake at given power until color sensor detects a pixel
    public void runIntake(double pwr) {
        intakeMotor.setPower(pwr);
        if (Objects.equals(getColor(colorLeft), "unknown")) {
            clawL.setPosition(1);
        } else if (Objects.equals(getColor(colorRight), "unknown")) {
            clawR.setPosition(1);
        } else if (!Objects.equals(getColor(colorLeft), "unknown")) {
            clawL.setPosition(.15);
        } else if (!Objects.equals(getColor(colorRight), "unknown")) {
            clawR.setPosition(.85);
        }
    }

    // ------ Function to Toggle Claw (Open/Close) ------ //
    public void toggleClaw(char servo) {
        switch (servo) {
            case 'L':
                clawL.setPosition(clawL.getPosition() == 1 ? 0.15 : 1);
                break;
            case 'R':
                clawR.setPosition(clawR.getPosition() == 0 ? 0.85 : 0);
                break;
            case 'C':
                clawR.setPosition(clawR.getPosition() == 0 ? 0.85 : 0);
                clawL.setPosition(clawL.getPosition() == 1 ? 0.15 : 1);
                break;
        }
    }

    // ------ Function to get Color ------ //
    public String getColor(ColorSensor CS) {
        int red = CS.red();
        int green = CS.green();
        int blue = CS.blue();
        String color;

        if (red > 200 && green > 200 && blue > 200) {
            color = "white";
        } else if (210 < red && green > 200 & blue < 160) {
            color = "yellow";
        } else if (green > red + blue) {
            color = "green";
        } else {
            color = "unknown";
        }

        return color;
    }

    // ------ Function to Power Slides ------ //
    public void powerSlides(float pwr) {
        leftPulley.setPower(pwr);
        rightPulley.setPower(pwr);
    }

    // ------ Function to Move to Delivery Pos. ------ //
    public void moveArmToDelivery() {
        wrist.setPosition(wristDeliveryPos);
        rightPulley.setTargetPosition(armDeliveryPos);
        leftPulley.setTargetPosition(armDeliveryPos);
    }

    // ------ Function to Move to Intake Pos. ------ //
    public void moveArmToIntake() {
        // TODO: Update Position Values
        wrist.setPosition(wristIntakePos);
        rightPulley.setTargetPosition(intakePos);
        leftPulley.setTargetPosition(intakePos);
    }

    // ------ Function to Reset Encoders in an Emergency ------ //
    public void resetEncoders() {
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }
}