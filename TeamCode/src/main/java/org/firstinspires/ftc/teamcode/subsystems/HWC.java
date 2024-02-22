package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.subsystems.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

/**
 * Stores and Declares all hardware devices &amp; related methods
 */
public class HWC {
    // ------ Computer Vision Labels ------ //
    private static final String[] LABELS = {
            "blue", "red"
    };

    // ------ Declare Servo Positions ------ //
    public static double passoverDeliveryPos = 0.2;
    public static double passoverIntakePos = 0.8;
    public static double wristDeliveryPos = 0.2;
    public static double wristIntakePos = 0.6;

    // ------ Declare Motors ------ //
    public DcMotorEx leftFront, rightFront, leftRear, rightRear, rightPulley, leftPulley, intakeMotor;

    // ------ Declare Servos ------ //
    public Servo wrist, clawR, clawL;

    // ------ Declare CRServos ------ //
    public CRServo passoverArmLeft, passoverArmRight;

    // ------ Declare Passover Encoders ------ //
    public AnalogInput passoverEncoderLeft, passoverEncoderRight;

    // ------ Declare Sensors ------ //
    public ColorRangeSensor colorLeft, colorRight;

    // ------ Declare Gamepads ------ //
    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad currentGamepad2 = new Gamepad();
    public Gamepad previousGamepad1 = new Gamepad();
    public Gamepad previousGamepad2 = new Gamepad();

    // ------ Declare Webcam ------ //
    public WebcamName webcam;

    // ------ Declare Roadrunner Drive ------ //
    public SampleMecanumDrive drive;

    // ------ ElapsedTime Variables ------ //
    public ElapsedTime time = new ElapsedTime();
    public ElapsedTime sleepTime = new ElapsedTime();

    // ------ Telemetry ------ //
    Telemetry telemetry;

    // ------ Declare TensorFlow Processor ------ //
    private TfodProcessor tfod;

    // ------ Computer Vision VisionPortal ------ //
    private VisionPortal visionPortal;

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
        clawL = hardwareMap.get(Servo.class, "clawL");
        clawR = hardwareMap.get(Servo.class, "clawR");
        wrist = hardwareMap.get(Servo.class, "wrist");

        // ------ Retrieve Continuous Rotation Servos ------ //
        passoverArmLeft = hardwareMap.get(CRServo.class, "passoverArmLeft");
        passoverArmRight = hardwareMap.get(CRServo.class, "passoverArmRight");

        // ------ Retrieve Passover CRServo Encoders ------ //
        passoverEncoderLeft = hardwareMap.get(AnalogInput.class, "passoverEncoderLeft");
        passoverEncoderRight = hardwareMap.get(AnalogInput.class, "passoverEncoderRight");

        // ------ Retrieve Sensors ------ //
        webcam = hardwareMap.get(WebcamName.class, "webcam");
        colorLeft = hardwareMap.get(ColorRangeSensor.class, "colorL");
        colorRight = hardwareMap.get(ColorRangeSensor.class, "colorR");

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
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ------ Set Motor Modes ------ //
        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftPulley.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightPulley.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    // ------ Function to Toggle Claw (Open/Close) ------ //
    public void toggleClaw(char servo) {
        switch (servo) {
            case 'L':
                clawL.setPosition(clawL.getPosition() == 1 ? 0.50 : 1);
                break;
            case 'R':
                clawR.setPosition(clawR.getPosition() == 0 ? 0.50 : 0);
                break;
            case 'C':
                clawR.setPosition(clawR.getPosition() == 0 ? 0.5 : 0);
                clawL.setPosition(clawL.getPosition() == 1 ? 0.5 : 1);
                break;
        }
    }

    public void betterSleep(int sleep) {
        sleepTime.reset();
        while (sleepTime.milliseconds() < sleep) {
            telemetry.addData("sleeping", "true");
            telemetry.update();
        }
        telemetry.addData("sleeping", "slept");
        telemetry.update();

    }

    public void sleepDeliver(int time) {
        intakeMotor.setPower(-0.3);
        betterSleep(time);
        intakeMotor.setPower(0);
    }

    // ------ Function to Power Slides ------ //
    public void powerSlides(float pwr) {
        leftPulley.setPower(pwr);
        rightPulley.setPower(pwr);
    }

    // ------ Function to Reset Motor Encoder Positions [EMERGENCY ONLY] ------ //
    public void resetEncoders() {
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void initTFOD(String TFOD_MODEL_ASSET) {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)
                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(webcam);

        builder.setCameraResolution(new Size(640, 480));

        builder.addProcessor(tfod);

        visionPortal = builder.build();

        tfod.setMinResultConfidence(0.75f);
    }

    private double telemetryTFOD() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());
        double x = 800;
        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;


            telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }
        return x;
    }

    public int cv() {
        double pos = telemetryTFOD(); //set it to a value depending on locaiton of obkect
        if (pos > 800) {
            return 0;
        } else if (pos < 100 && pos > 0) {
            return 1;
        } else if (pos < 400) {
            return 2;
        } else if (pos < 800) {
            return 3;
        } else {
            return 0;
        }
    }
}