package org.firstinspires.ftc.teamcode;

import static org.ironlions.sovereign.pathfinding.actualization.UtilKt.rpmToVelocity;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.ironlions.common.geometry.Point;
import org.ironlions.sovereign.components.BasicComponent;
import org.ironlions.sovereign.components.Component;
import org.ironlions.sovereign.opmode.OpModeProvider;
import org.ironlions.sovereign.pathfinding.actualization.ActualizationContext;
import org.ironlions.sovereign.pathfinding.actualization.LocalizationMethod;
import org.ironlions.sovereign.pathfinding.algorithms.AStar;
import org.ironlions.sovereign.pathfinding.fitting.tree.grid.GridTreeFitter;
import org.ironlions.sovereign.pathfinding.pipeline.PathfindingPipeline;

// TODO: These values were ripped from the Quickstart's `DriveConstants.java`. Replace these with
//  real values!
public class Drivetrain extends BasicComponent {
    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
    LocalizationMethod localizationMethod = new LocalizationMethod.Encoder();
    double ticksPerRevolution = 1.0;
    double maxRevolutionsPerMinute = 1.0;
    double wheelRadius = 2.0;
    double gearRatio = 1.0;
    double trackWidth = 1.0;
    double kVelocity = 1.0 / rpmToVelocity(maxRevolutionsPerMinute, gearRatio, wheelRadius);
    double kAcceleration = 0.0;
    double kStatic = 0.0;
    double maxVelocity = 30.0;
    double maxAcceleration = 30.0;
    double maxAngleVelocity = Math.toRadians(60.0);
    double maxAngleAcceleration = Math.toRadians(60.0);
    private final ActualizationContext actualizationContext = new ActualizationContext(logoDirection, usbDirection, localizationMethod, ticksPerRevolution, maxRevolutionsPerMinute, wheelRadius, gearRatio, trackWidth, kVelocity, kAcceleration, kStatic, maxVelocity, maxAcceleration, maxAngleVelocity, maxAngleAcceleration);
    private final PathfindingPipeline pipeline;

    public Drivetrain(@NonNull BasicComponent.Concrete actor, @NonNull OpModeProvider belonging, @Nullable Component parent) {
        super(belonging, parent, "Drivetrain");
        this.pipeline = new PathfindingPipeline(actor, actualizationContext, new GridTreeFitter.Builder().resolution(3), new AStar.Builder());
    }

    public void goToPoint(Point point) {
        pipeline.cycle(point);

        throw new RuntimeException("Trajectory following not yet implemented for RoadRunner 1.0.0.");
    }
}
