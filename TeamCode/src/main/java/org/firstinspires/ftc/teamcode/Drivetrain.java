package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.VelConstraint;

import org.ironlions.common.geometry.Point;
import org.ironlions.sovereign.components.BasicComponent;
import org.ironlions.sovereign.components.Component;
import org.ironlions.sovereign.opmode.OpModeProvider;
import org.ironlions.sovereign.pathfinding.actualization.ActualizationContext;
import org.ironlions.sovereign.pathfinding.actualization.ActionFactory;
import org.ironlions.sovereign.pathfinding.algorithms.AStar;
import org.ironlions.sovereign.pathfinding.fitting.tree.grid.GridTreeFitter;
import org.ironlions.sovereign.pathfinding.pipeline.PathfindingPipeline;

// TODO: Replace all 0.0s and RuntimeExceptions with real, honest values.
public class Drivetrain extends BasicComponent implements ActionFactory {
    private static final double EPS = 0.0;
    private static final double BEGIN_END_VAL = 0.0;
    private static final double DISP_RESOLUTION = 0.0;
    private static final double ANG_RESOLUTION = 0.0;
    private static final TurnConstraints BASE_TURN_CONSTRAINTS = new TurnConstraints(0.0, 0.0, 0.0);
    private static final VelConstraint BASE_VEL_CONSTRAINT = new TranslationalVelConstraint(0.0);
    private static final AccelConstraint BASE_ACCEL_CONSTRAINT = new ProfileAccelConstraint(0.0, 0.0);
    private final ActualizationContext actualizationContext = new ActualizationContext(this, this, BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT, BASE_TURN_CONSTRAINTS, EPS, BEGIN_END_VAL, DISP_RESOLUTION, ANG_RESOLUTION);
    private final PathfindingPipeline pipeline;

    public Drivetrain(@NonNull BasicComponent.Concrete actor, @NonNull OpModeProvider belonging, @Nullable Component parent) {
        super(belonging, parent, "Drivetrain");
        this.pipeline = new PathfindingPipeline(actor, actualizationContext, new GridTreeFitter.Builder().resolution(3), new AStar.Builder());
    }

    public void goToPoint(Point point) {
        pipeline.cycle(point);

        throw new RuntimeException("Trajectory following not yet implemented for RoadRunner 1.0.0.");
    }

    @NonNull
    @Override
    public Action make(@NonNull TimeTrajectory timeTrajectory) {
        // TODO: Base off of old MecanumDrive/TankDrive.
        throw new RuntimeException("Unimplemented");
    }

    @NonNull
    @Override
    public Action make(@NonNull TimeTurn timeTurn) {
        // TODO: Base off of old MecanumDrive/TankDrive.
        throw new RuntimeException("Unimplemented");
    }
}
