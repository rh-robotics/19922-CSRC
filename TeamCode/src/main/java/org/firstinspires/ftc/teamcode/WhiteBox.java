package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import org.ironlions.common.geometry.Measurement;
import org.ironlions.common.geometry.Point;
import org.ironlions.common.geometry.Region;
import org.ironlions.sovereign.components.BasicComponent;
import org.ironlions.sovereign.components.Component;
import org.ironlions.sovereign.opmode.MakeAvailable;
import org.ironlions.sovereign.opmode.OpModeProvider;
import org.ironlions.sovereign.opmode.OpModeType;

import java.util.Arrays;
import java.util.concurrent.ThreadLocalRandom;

/**
 * This is an OpMode for a white boxed robot that makes the arm extend itself to random points,
 * while the robot moves to the center of the field.
 * </br>
 * This uses a pretty limited subset of Sovereign functionality, but pathfinding and the component
 * system are indeed used.
 */
@MakeAvailable(type = OpModeType.AUTON)
public class WhiteBox extends BasicComponent.Concrete {
    private Drivetrain drivetrain;
    //private Arm arm;
    private int cnt = 0;

    public WhiteBox(@NonNull OpModeProvider belonging, @Nullable Component parent) {
        super(belonging, parent, "WhiteBox", new Region(new Point(new Measurement.Centimeters(0), new Measurement.Centimeters(0), new Measurement.Centimeters(0)), new Point(new Measurement.Centimeters(1), new Measurement.Centimeters(1), new Measurement.Centimeters(1))));
    }

    @Override
    public void init() {
        drivetrain = new Drivetrain(this, getBelonging(), this);
        // arm = new Arm(getBelonging(), this);
        components.addAll(Arrays.asList(drivetrain)); //, arm));
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        telemetry.addData("Current Time", getTime());

        /* if (arm.isAtTarget()) {
            telemetry.addLine("Arm target reached, setting new random position.");
            arm.setExtensionTarget(ThreadLocalRandom.current().nextInt(800, 3900 + 1));
        } */

        if (!drivetrain.isBusy() && cnt != 0) {
            stop();
        }

        if (!drivetrain.isBusy()) {
            drivetrain.goToPoint(new Point(new Measurement.Centimeters(0.0), new Measurement.Centimeters(0.0)));
        }

        cnt = 1;
    }
}
