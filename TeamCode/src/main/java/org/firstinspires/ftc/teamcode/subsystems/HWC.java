package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.kinematics.Odometry;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.vision.tensorflowFPA;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvInternalCamera;


/**
 * Stores and Declares all hardware devices &amp; related methods
 */
public class HWC {
    // ------ Declare Motors ------ //
    public DcMotorEx leftFront, rightFront, leftRear, rightRear, rightPulley, leftPulley, intakeMotor;

    // ------ Declare Servos ------ //
    public Servo intakeL, wristL, wristR, clawR, clawL,passoverArmLeft, passoverArmRight, droneKicker, droneAimer;

    // ------ Declare Sensors ------ //
    public ColorSensor colorLeft, colorRight;
    //public DcMotorEx xWheel, yWheel;

    // ------ Declare Continuous Rotation Servos ------ //
    //public CRServo passoverArmLeft, passoverArmRight;

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
        intakeL = hardwareMap.get(Servo.class, "intakeL");
        clawL = hardwareMap.get(Servo.class, "clawL");
        clawR = hardwareMap.get(Servo.class, "clawR");
        wristL = hardwareMap.get(Servo.class, "wristL");
        wristR = hardwareMap.get(Servo.class, "wristR");
        // droneAimer = hardwareMap.get(Servo.class, "droneAim");
        // droneKicker = hardwareMap.get(Servo.class, "droneKick");

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
        if (returnColor(colorLeft) == "unknown") {
            clawL.setPosition(1);
        } else if (returnColor(colorRight) == "unknown") {
            clawR.setPosition(1);
        } else if (returnColor(colorLeft) != "unknown") {
            clawL.setPosition(.15);

        } else if (returnColor(colorRight) != "unknown") {
            clawR.setPosition(.85);

        }
    }

    public void oldIntake(double pwr) {
        intakeMotor.setPower(pwr);
    }

    public void changeIntakePos(double pos) {
        intakeL.setPosition(pos);
        // intakeR.setPosition(pos);
    }

    public void toggleClaw(char servo) {
        // TODO: Check Servo Positions
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

    /*public char checkIntakeSensors() {
        //add new sensor if used
        if (buttonL.isPressed() && buttonR.isPressed()) {
            return 'B';
        } else if (buttonL.isPressed()) {
            return 'L';
        } else if (buttonR.isPressed()) {
            return 'R';
        } else return '0';
    }*/

    public String returnColor(ColorSensor CS) {
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

    /*  public void slapDrone(int pos){
          droneKicker.setPosition(pos);

      }
      public void aimDrone(int pos){
          droneAimer.setPosition(pos);
      }
  */
  /*  public void fullIntake() {
        changeIntakePos(intakePos);
        while (checkIntakeSensors() != 'B') {
            runIntake(1);
        }
        toggleClaw('C');
    }
*/
    public void slideControl(float pwr) {
        leftPulley.setPower(pwr);
        rightPulley.setPower(pwr);
    }

    public void moveArmToDelivery() {
        wristL.setPosition(elbowDeliveryPos);
        wristR.setPosition(elbowDeliveryPos);
        rightPulley.setTargetPosition(armDeliveryPos);
        leftPulley.setTargetPosition(armDeliveryPos);
    }

    public void moveWrist(int posL, int posR) {
        wristL.setPosition(posL);
        wristR.setPosition(posR);
    }

    public void movePassover(int pos1, int pos2) {
        passoverArmRight.setPosition(pos1);
        passoverArmLeft.setPosition(pos2);
    }

    public void resetEncoders() {
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void betterSleep(int milliseconds){
        time.reset();
        while (time.milliseconds() > milliseconds){}
       // telemetry.addData("slept for ", milliseconds);
    }
    public void sleepDrive(int time){
        leftFront.setPower(0.3);
        rightFront.setPower(0.3);
        leftRear.setPower(0.3);
        rightRear.setPower(0.3);
        betterSleep(time);
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

    }

//
public void odoDrive(int distance){
        leftRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER); // y axis
        rightRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER); // x axis
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER); // x axis
        while (leftFront.getCurrentPosition() * -1 < distance && rightRear.getCurrentPosition() < distance){
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

public void odoTurn(int degrees){

}


    public int cv(){
         double pos = 0; //set it to a value depending on locaiton of obkect
         if (pos > 800){
             return 0;
         }
         else if (pos < 100){
             return 1;
         }
         else if (pos < 400){
             return 2;
        }
         else if (pos < 800){
             return 3;
         }
         else {return 0;}
    }
}