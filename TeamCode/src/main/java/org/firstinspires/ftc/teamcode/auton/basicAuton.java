package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.HWC;
import org.firstinspires.ftc.teamcode.subsystems.fsm.Edge;
import org.firstinspires.ftc.teamcode.subsystems.fsm.State;
import org.firstinspires.ftc.teamcode.subsystems.fsm.StateMachine;

/**
 * Main Autonomous OpMode for the robot
 */
@Autonomous
public class basicAuton extends LinearOpMode {







    public void runOpMode() {
        HWC robot = new HWC(hardwareMap, telemetry); // Declare HardwareClass Object
        int pos = robot.cv();
        waitForStart();
        boolean delivered = false;
        while (!delivered) {
            if (pos == 1) {
            robot.sleepDrive(500);
            robot.rightRear.setPower(0.2);
           // robot.leftFront

            }
        }

    }
}
