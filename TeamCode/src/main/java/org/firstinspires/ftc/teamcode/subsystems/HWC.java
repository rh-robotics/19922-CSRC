package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

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
    private static final String[] LABELS = {"blue", "red"};
    public static double passoverDeliveryPos = 0.2;
    public static double passoverIntakePos = 0.8;
    public static double wristDeliveryPos = 0.2;
    public static double wristIntakePos = 0.6;
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
    // ------ Position Variables ------ //
    //TODO: UPDATE WITH REAL NUMBERS ONCE TESTED
    public ElapsedTime sleepTime = new ElapsedTime();
    // ------ Telemetry ------ //
    private Telemetry telemetry;
    // ------ Declare TensorFlow Processor ------ //
    private TfodProcessor tfod;
    // ------ Computer Vision VisionPortal ------ //
    private VisionPortal visionPortal;
    private boolean roadrunner;

    /**
     * Constructor for HWC, declares all hardware components
     *
     * @param hardwareMap HardwareMap - Used to retrieve hardware devices
     * @param telemetry   Telemetry - Used to add telemetry to driver hub
     */
    public HWC(@NonNull HardwareMap hardwareMap, Telemetry telemetry, boolean roadrunner) {
        this.telemetry = telemetry;
        this.roadrunner = roadrunner;

        if (roadrunner) {
            // ------ Declare RR Drivetrain ------ //
            drive = new SampleMecanumDrive(hardwareMap);
        } else {
            // ------ Retrieve Drive Motors ------ //
            leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
            rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
            leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
            rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        }

        // ------ Retrieve Other Motors ------ //
        rightPulley = hardwareMap.get(DcMotorEx.class, "pulleyR");
        leftPulley = hardwareMap.get(DcMotorEx.class, "pulleyL");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        // ------ Retrieve Servos ------ //
        intakeArm = hardwareMap.get(Servo.class, "intakeArm");
        clawL = hardwareMap.get(Servo.class, "clawL");
        clawR = hardwareMap.get(Servo.class, "clawR");
        wrist = hardwareMap.get(Servo.class, "wrist");

        // ------ Retrieve Continuous Rotation Servos ------ //
        passoverArmLeft = hardwareMap.get(Servo.class, "passoverArmLeft");
        passoverArmRight = hardwareMap.get(Servo.class, "passoverArmRight");

        // ------ Retrieve Sensors ------ //
        webcam = hardwareMap.get(WebcamName.class, "webcam");
        colorLeft = hardwareMap.get(ColorSensor.class, "colorL");
        colorRight = hardwareMap.get(ColorSensor.class, "colorR");

        // ------ Set Motor Directions ------ //
        if (!roadrunner) {
            leftFront.setDirection(DcMotorEx.Direction.FORWARD);
            rightFront.setDirection(DcMotorEx.Direction.FORWARD);
            leftRear.setDirection(DcMotorEx.Direction.FORWARD);
            rightRear.setDirection(DcMotorEx.Direction.FORWARD);
        }

        leftPulley.setDirection(DcMotorEx.Direction.REVERSE);


        // ------ Set Motor Brake Modes ------ //
        if (!roadrunner) {
            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ------ Set Motor Modes ------ //
        if (!roadrunner) {
            leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            leftRear.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            rightRear.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftPulley.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightPulley.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    // ------ Function to Run Intake ------ //
    // Runs intake at given power until color sensor detects a pixel
    public void runIntake(double pwr) {
        intakeMotor.setPower(pwr);
        if (intakeDetection(colorLeft) == 0 && intakeDetection(colorRight) == 0) {
            clawL.setPosition(1);
            clawR.setPosition(1);
            intakeMotor.setPower(0);
        } else if (intakeDetection(colorLeft) == 1) {
            clawL.setPosition(1);

        } else if (intakeDetection(colorRight) == 0) {
            clawR.setPosition(1);
        }

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

    public int intakeDetection(ColorSensor CS) {
        return (CS.argb() == 0) ? 1 : 0;
    }

    // ------ Function that Allows For Sleeping in OpModes ------ //
    public void elapsedTimeSleep(int milliseconds) {
        sleepTime.reset();
        while (sleepTime.milliseconds() < milliseconds) {
            telemetry.addData("sleeping", "true");
            telemetry.update();
        }
        telemetry.addData("sleeping", "slept");
        telemetry.update();
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

    // ------ Function to Initialize TensorFlow Object Detection ------ //
    public void initTFOD(String TFOD_MODEL_ASSET) {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder().setModelAssetName(TFOD_MODEL_ASSET).setModelLabels(LABELS).build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(webcam);

        builder.setCameraResolution(new Size(640, 480));

        builder.addProcessor(tfod);

        visionPortal = builder.build();

        tfod.setMinResultConfidence(0.75f);
    }

    // ------ Function to add Telemetry for TensorFlow Object Detection ------ //
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

    // ------ Function to Detect Team Element with TensorFlow Object Detection ------ //
    public boolean detectElement() {
        return telemetryTFOD() < 800;
    }

    @NonNull
    @Override
    public String toString() {
        StringBuilder builder = new StringBuilder("HWC {");

        if (roadrunner) {
            builder.append("\n\tbusy = ").append(drive.isBusy());
        } else {
            builder.append("\n\tleftFront = ").append(leftFront.getPower()).append("\n\trightFront = ").append(rightFront.getPower()).append("\n\tleftRear = ").append(leftRear.getPower()).append("\n\trightRear = ").append(rightRear.getPower());
        }

        builder.append("\n\trightPulley = ").append(rightPulley.getPower()).append("\n\tleftPulley = ").append(leftPulley.getPower()).append("\n\tintakeMotor = ").append(intakeMotor.getPower()).append("\n\tintakeArm = ").append(intakeArm.getPosition()).append("\n\twrist = ").append(wrist.getPosition()).append("\n\tclawR = ").append(clawR.getPosition()).append("\n\tclawL = ").append(clawL.getPosition()).append("\n\tpassoverArmLeft = ").append(passoverArmLeft.getPosition()).append("\n\tpassoverArmRight = ").append(passoverArmRight.getPosition()).append("\n}");

        return builder.toString();
    }
}