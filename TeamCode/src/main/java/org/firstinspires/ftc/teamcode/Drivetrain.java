package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.ironlions.common.geometry.Point;
import org.ironlions.sovereign.components.BasicComponent;
import org.ironlions.sovereign.components.Component;
import org.ironlions.sovereign.opmode.OpModeProvider;
import org.ironlions.sovereign.pathfinding.actualization.ActualizationContext;
import org.ironlions.sovereign.pathfinding.algorithms.AStar;
import org.ironlions.sovereign.pathfinding.fitting.tree.grid.GridTreeFitter;
import org.ironlions.sovereign.pathfinding.pipeline.PathfindingPipeline;

import org.firstinspires.ftc.teamcode.subsystems.roadrunner.drive.KitMecanumDrive;

// TODO: These values were ripped from the Quickstart's `DriveConstants.java`. Replace these with
//  real values!
public class Drivetrain extends BasicComponent {
    private final KitMecanumDrive kitMecanumDrive;
    private final ActualizationContext actualizationContext;
    private final PathfindingPipeline pipeline;

    public Drivetrain(@NonNull BasicComponent.Concrete actor, @NonNull OpModeProvider belonging, @Nullable Component parent) {
        super(belonging, parent, "Drivetrain");

        this.kitMecanumDrive = new KitMecanumDrive(hardwareMap);
        this.actualizationContext = new ActualizationContext(kitMecanumDrive); // logoDirection, usbDirection, localizationMethod, ticksPerRevolution, maxRevolutionsPerMinute, wheelRadius, gearRatio, trackWidth, kVelocity, kAcceleration, kStatic, maxVelocity, maxAcceleration, maxAngleVelocity, maxAngleAcceleration);
        this.pipeline = new PathfindingPipeline(actor, actualizationContext, new GridTreeFitter.Builder().resolution(3), new AStar.Builder());
    }

    public void goToPoint(Point point) {
        Trajectory trajectory = pipeline.cycle(point, new Pose2d());
        kitMecanumDrive.followTrajectory(trajectory);
    }

    public boolean isBusy() {
        return kitMecanumDrive.isBusy();
    }
}
