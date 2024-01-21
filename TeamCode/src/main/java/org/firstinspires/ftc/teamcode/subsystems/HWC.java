package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.tfod.TfodParameters.CurrentGame.LABELS;

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
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.subsystems.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
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

    // ------ Declare TensorFlow Processor ------ //
    private TfodProcessor tfod;

    // ------ Computer Vision VisionPortal ------ //
    private VisionPortal visionPortal;

    // ------ Computer Vision Labels ------ //
    private static final String[] LABELS = {
            "blue", "red"
    };

    // ------ Declare Webcam ------ //
    public WebcamName webcam;

    // ------ Declare Roadrunner Drive ------ //
    public SampleMecanumDrive drive;

    // ------ ElapsedTime Variable ------ //
    public ElapsedTime time = new ElapsedTime();

    // ------ Telemetry ------ //
    Telemetry telemetry;

    public ElapsedTime sleepTime = new ElapsedTime();
    // ------ Position Variables ------ //
    //TODO: UPDATE WITH REAL NUMBERS ONCE TESTED

    public static double passoverDeliveryPos = 0.2;
    public static double passoverIntakePos = 0.8;
    public static double wristDeliveryPos = 0.2;
    public static double wristIntakePos = 0.5;

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

        // ------- Odemetry ------//
        // xWheel = hardwareMap.get(DcMotorEx.class, "odoX");
        //yWheel = hardwareMap.get(DcMotorEx.class,"odoY");

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

        // ------ Retrieve Continuous Rotation Servos ------ //
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
    public void betterSleep(int sleep){
        sleepTime.reset();
        while (sleepTime.milliseconds() < sleep){
            telemetry.addData("sleeping", "true");
            telemetry.update();}
        telemetry.addData("sleeping", "slept");
        telemetry.update();

    }
    public void sleepDeliver(int time){
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

    // ------ Function to Drive Given Distance Using Odometry ------ //
    public void odoDrive(int distance) {
        leftRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER); // y axis
        rightRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER); // x axis
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER); // x axis
        while (leftRear.getCurrentPosition() > distance) {
            leftFront.setPower(0.3);
            rightFront.setPower(0.3);
            leftRear.setPower(0.3);
            rightRear.setPower(0.3);
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }

    // ------ Function to Strafe Given Distance Using Odometry ------ //
    public void odoStrafeLeft(int distance) {
        odoDrive(500);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // y axis
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // x axis
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER); // x axis


                while(leftRear.getCurrentPosition() < distance) {
            leftFront.setPower(-0.3);
            rightFront.setPower(0.3);
            leftRear.setPower(0.3);
            rightRear.setPower(-0.3);}
    }
    public void odoStrafeRight(int distance) {
        odoDrive(500);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // y axis
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // x axis
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER); // x axis


        while(Math.abs(leftRear.getCurrentPosition()) < distance) {
            leftFront.setPower(0.3);
            rightFront.setPower(-0.3);
            leftRear.setPower(-0.3);
            rightRear.setPower(0.3);}
    }

    public void odoTurnRight(){
        leftFront.setPower(0.3);
        rightFront.setPower(-0.3);
        leftRear.setPower(-0.3);
        rightRear.setPower(0.3);
    }

    // ------ Function to Turn Given Degrees Using Odometry ------ //
    public void odoTurn(int degrees)
        {
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