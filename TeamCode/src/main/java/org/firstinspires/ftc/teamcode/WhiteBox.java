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
public class WhiteBox extends BasicComponent {
    // private Drivetrain drivetrain;
    private Arm arm;

    public WhiteBox(@NonNull OpModeProvider belonging, @Nullable Component parent) {
        super(belonging, parent, "WhiteBox");
    }

    @Override
    public void init() {
        //drivetrain = new Drivetrain(this, getBelonging(), this);
        arm = new Arm(getBelonging(), this);
        components.addAll(Arrays.asList(arm)); //drivetrain, arm));
    }

    @Override
    public void start() {
        //drivetrain.goToPoint(new Point(new Measurement.Centimeters(0.0), new Measurement.Centimeters(0.0)));
    }

    @Override
    public void loop() {
        telemetry.addData("Current Time", getTime());

        if (arm.isAtTarget()) {
            telemetry.addLine("Arm target reached, setting new random position.");
            arm.setExtensionTarget(ThreadLocalRandom.current().nextInt(300, 5000 + 1));
        }
    }
}
