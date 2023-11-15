package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.kinematics.Odometry;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
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
    public TouchSensor buttonL, buttonR;
    public Servo passoverArmLeft, passoverArmRight;


    // Other Variables
    Telemetry telemetry;
    public ElapsedTime time = new ElapsedTime();
    public SampleMecanumDrive drive;
    // Declare Position Variables
    //TODO: UPDATE WITH REAL NUMBERS ONCE TESTED
    double intakePos = 5; //made up number, needs to be tested and actually found
    double armPos = 6; // Another made up variable
    double openClawPos = 5;
    double closedClawPos = 0;

    // Other Variables

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
        rightPulley = hardwareMap.get(DcMotorEx.class, "pulleyR");
        leftPulley = hardwareMap.get(DcMotorEx.class, "pulleyL");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        // Declare Servos
        intakeL = hardwareMap.get(Servo.class, "IntakeL");
        intakeR = hardwareMap.get(Servo.class, "IntakeR");
        clawL = hardwareMap.get(Servo.class, "ClawL");
        clawR = hardwareMap.get(Servo.class, "ClawR");
        passoverArmLeft = hardwareMap.get(Servo.class, "passoverArmLeft");
        passoverArmRight = hardwareMap.get(Servo.class, "passoverArmRight");

        // Declare Sensors
        buttonL = hardwareMap.get(TouchSensor.class, "buttonL");
        buttonR = hardwareMap.get(TouchSensor.class, "buttonR");

        // Set the direction of motors
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftRear.setDirection(DcMotorEx.Direction.FORWARD);
        rightRear.setDirection(DcMotorEx.Direction.FORWARD);

        // Set motors to break when power = 0
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

    public void runIntake(double pwr) {
        intakeMotor.setPower(pwr);

    }

    public void changeIntakePos(double pos) {
        intakeL.setPosition(pos);
        intakeR.setPosition(pos);
    }

    public void toggleClaw(char servo) {
        if (servo == 'L') {
            if (clawL.getPosition() == openClawPos) {
                clawL.setPosition(closedClawPos);
            } else if (servo == 'R') {
                if (clawR.getPosition() == openClawPos) {
                    clawR.setPosition(closedClawPos);
                } else if (servo == 'C') {
                    clawR.setPosition(closedClawPos);
                    clawL.setPosition(closedClawPos);
                } else if (servo == 'O') {
                    clawR.setPosition(openClawPos);
                    clawL.setPosition(openClawPos);
                } else {
                    if (clawR.getPosition() == openClawPos) {
                        clawR.setPosition(closedClawPos);
                    }
                }
            }
        }
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

    public void fullIntake() {
        changeIntakePos(intakePos);
        while (checkIntakeSensors() != 'B') {
            runIntake(1);
        }

    }
}