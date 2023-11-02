package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.kinematics.Odometry;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorTouch;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.roadrunner.drive.SampleMecanumDrive;

/**
 * Stores and Declares all hardware devices &amp; related methods
 */
public class HWC {
    // Declare empty variables for robot hardware
    public DcMotorEx leftFront, rightFront, leftRear, rightRear, rightPulley, leftPulley, intakeMotor;
    public Servo intakeL, intakeR, elbowL, elbowR, clawR, clawL;
    public SensorTouch buttonL, buttonR;


    // Other Variables
    Telemetry telemetry;
    public ElapsedTime time = new ElapsedTime();
    public SampleMecanumDrive drive;
    // Declare Position Variables
    double intakePos = 5; //made up number, needs to be tested and actually found
    double armPos = 6; // Another made up variable

    /**
     * Constructor for HWC, declares all hardware components
     *
     * @param hardwareMap HardwareMap - Used to retrieve hardware devices
     * @param telemetry   Telemetry - Used to add telemetry to driver hub
     */
    public HWC(@NonNull HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        drive = new SampleMecanumDrive(hardwareMap);





        // Declare driving motors
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

        // Declare other motors
        rightPulley = hardwareMap.get(DcMotorEx.class, "R_Pulley");
        leftPulley = hardwareMap.get(DcMotorEx.class, "L_Pulley");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        // Declare Servos
        intakeL = hardwareMap.get(Servo.class, "Intake_L");
        intakeR = hardwareMap.get(Servo.class, "Intake_R");
        elbowL = hardwareMap.get(Servo.class, "Elbow_L");
        elbowR = hardwareMap.get(Servo.class, "Elbow_R");
        clawL = hardwareMap.get(Servo.class, "Claw_L");
        clawR = hardwareMap.get(Servo.class, "Claw_R");
        // Declare Sensors
        buttonL = hardwareMap.get(SensorTouch.class, "L_Button");
        buttonR = hardwareMap.get(SensorTouch.class, "R_Button");


        // Set the direction of motors
        // TODO: UPDATE VALUES WITH NEW BOT
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftRear.setDirection(DcMotorEx.Direction.FORWARD);
        rightRear.setDirection(DcMotorEx.Direction.FORWARD);

        // Set motors to break when power = 0
        // TODO: REMOVE IF THIS BEHAVIOUR IS NOT DESIRED ON NEW BOT
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightPulley.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftPulley.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Run motors using encoder, so that we can move accurately.
        leftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightPulley.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftPulley.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
    // TODO: ADD ANY HARDWARE RELATED FUNCTIONS BELOW

    public void runIntake(double pwr){
        intakeR.setPosition(intakePos);
        intakeMotor.setPower(pwr);

    }
}