package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/**
 * Stores and Declares all hardware devices &amp; related methods
 */
public class HWC {
    // ------ Declare Hardware ------ //
    public DcMotorEx leftFront, rightFront, leftRear, rightRear, rightPulley, leftPulley, intakeMotor;
    public Servo intakeL, intakeR, wristL, wristR, clawR, clawL, droneKicker, droneAimer;
    public TouchSensor buttonL, buttonR;
    public ColorSensor colorLeft, colorRight;
    public CRServo passoverArmLeft, passoverArmRight;
    public WebcamName webcam;
    public ElapsedTime time = new ElapsedTime();
    public SampleMecanumDrive drive;

    // ------ Telemetry ------ //
    Telemetry telemetry;

    // ------ Position Variables ------ //
    //TODO: UPDATE WITH REAL NUMBERS ONCE TESTED

    int intakePos = 5;
    int armDeliveryPos = 6;
    int armRetractedPos = 0;
    double elbowDeliveryPos = 20;

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

        // ------ Retrieve Hardware Devices ------ //

        // Drive Motors
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

        // Other Motors
        rightPulley = hardwareMap.get(DcMotorEx.class, "pulleyR");
        leftPulley = hardwareMap.get(DcMotorEx.class, "pulleyL");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        // Servos
        intakeL = hardwareMap.get(Servo.class, "intakeL");
        intakeR = hardwareMap.get(Servo.class, "intakeR");
        clawL = hardwareMap.get(Servo.class, "clawL");
        clawR = hardwareMap.get(Servo.class, "clawR");
        wristL = hardwareMap.get(Servo.class, "wristL");
        wristR = hardwareMap.get(Servo.class, "wristR");
        droneAimer = hardwareMap.get(Servo.class, "droneAim");
        droneKicker = hardwareMap.get(Servo.class, "droneKick");

        // Continuous Rotation Servos
        passoverArmLeft = hardwareMap.get(CRServo.class, "passoverArmLeft");
        passoverArmRight = hardwareMap.get(CRServo.class, "passoverArmRight");

        // Sensors
        buttonL = hardwareMap.get(TouchSensor.class, "buttonL");
        buttonR = hardwareMap.get(TouchSensor.class, "buttonR");
        webcam = hardwareMap.get(WebcamName.class, "webcam");
        colorLeft =hardwareMap.get(ColorSensor.class,"colorL");
        colorRight =hardwareMap.get(ColorSensor.class,"colorR");

        // ------ Set Motor Directions ------ //
        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftRear.setDirection(DcMotorEx.Direction.FORWARD);
        rightRear.setDirection(DcMotorEx.Direction.FORWARD);
        leftPulley.setDirection(DcMotorEx.Direction.REVERSE);
        passoverArmLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        passoverArmRight.setDirection(DcMotorSimple.Direction.FORWARD);

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

    /**
     * Initialize the AprilTag processor.
     */
    public static void initAprilTag(AprilTagProcessor aprilTag, VisionPortal visionPortal, HWC robot) {

        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagOutline(true)
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(robot.webcam); // Set Camera to webcam
        builder.setCameraResolution(new Size(640, 480)); // Set Camera Resolution
        builder.enableLiveView(true); // Enable preview on Robot Controller (Driver Station)
        builder.addProcessor(aprilTag); // Set and enable the processor
        visionPortal = builder.build(); // Build the Vision Portal, using the above settings
    }

    public void runIntake(double pwr) {
        intakeMotor.setPower(pwr);
        if (returnColor(colorLeft)=="unknown"){
            clawL.setPosition(1);
        }
        else if (returnColor(colorRight)=="unknown"){
            clawR.setPosition(1);
        }
        else if (returnColor(colorLeft) != "unknown"){
            clawL.setPosition(.15);

        }
        else if (returnColor(colorRight) != "unknown"){
            clawR.setPosition(.85);

        }
    }

    public void changeIntakePos(double pos) {
        intakeL.setPosition(pos);
        intakeR.setPosition(pos);
    }

    public void toggleClaw(char servo) {
        if (servo == 'L') {
            // 1 is open, 0.5 is closed. If clawL is open, close it. Otherwise, open it.
            if (clawL.getPosition() == 1) {
                clawL.setPosition(0.15);
            } else {
                clawL.setPosition(1);
            }
        } else if (servo == 'R') {
            if (clawR.getPosition() == 0) {
                clawR.setPosition(0.85);
            } else {
                clawR.setPosition(0);
            }
        } else if (servo == 'C') {
            clawR.setPosition(0.85);
            clawL.setPosition(0.15);
        }
        betterSleep(2000);
    }

    public char checkIntakeSensors() {
        //add new sensor if used
        if (buttonL.isPressed() && buttonR.isPressed()) {
            return 'B';
        } else if (buttonL.isPressed()) {
            return 'L';
        } else if (buttonR.isPressed()) {
            return 'R';
        } else return '0';
    }
    public String returnColor(ColorSensor CS) {
        int red = CS.red();
        int green = CS.green();
        int blue = CS.blue();
        String color;

        if (red > 200 && green > 200 && blue > 200) {
            color = "white";
        } else if (210 < red && green >200 & blue < 160) {
            color = "yellow";}

        else if (green > red+blue){
            color = "green";}
        else {
            color = "unknown";}

        return color;
    }

    public void testColorSensor(ColorSensor CS){
        telemetry.addData("R, G, B", CS.argb());
    }


    public void slapDrone(int pos){
    droneKicker.setPosition(pos);

    }
    public void aimDrone(int pos){
        droneAimer.setPosition(pos);
    }

    public void fullIntake() {
        changeIntakePos(intakePos);
        while (checkIntakeSensors() != 'B') {
            runIntake(1);
        }
        toggleClaw('C');
    }

    public void manualArm(float pwr) {
        leftPulley.setPower(pwr);
        rightPulley.setPower(pwr);
    }

    public void betterSleep(int milliseconds) {
        time.reset();

        while (time.milliseconds() > milliseconds) {
            //noinspection UnnecessaryContinue
            continue;
        }

        telemetry.addData("slept for ", milliseconds);
    }

    public void moveArmToDelivery() {
        wristL.setPosition(elbowDeliveryPos);
        wristR.setPosition(elbowDeliveryPos);
        rightPulley.setTargetPosition(armDeliveryPos);
        leftPulley.setTargetPosition(armDeliveryPos);
    }

    public void movePassover(float pwr) {
        passoverArmRight.setPower(pwr);
    }
}